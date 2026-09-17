# Balloon Free Page Reporting (FPR) 设计文档

> 对应实现：`Balloon/sys/reporting.c`（版本 58600 起）
> 系列提交：210aaf80 → 50fe3ebd → 74f4ab2c → 22ad5876 → febfb2e2

## 1. 目标与方案

在 Windows guest 上实现 `VIRTIO_BALLOON_F_PAGE_REPORTING`（virtio 1.x §5.5.6.7）：

- **report-then-hold**：上报后的页保持驱动持有（不归还 OS），guest 无法触碰，host 侧
  discard 效果持续有效；guest 需要内存时才归还（水位/低内存/commit 触发）。
  依据：report-then-release 在 Windows 不可行（归还页被 MM zeroing 触碰，host 侧
  回收效果立即消失）；hold 语义已由 virtio-comment 讨论确认在现有 spec 之内
  （"reuse the reported free pages when needed" 中的归还仅为示例），无需新 feature bit。
- **2MB 对齐上报**（大小与边界）：对齐 host 侧 THP 回收粒度与 Linux pageblock_order
  默认粒度；spec "SHOULD attempt to report large pages rather than smaller ones"。
- 上游系列定位：机制 → 测试 → 参数化 → 响应性 → commit 保护，五笔可拆分提交。

## 2. 线程模型

```
BalloonRoutine（复用现有 balloon worker 线程）
  └── KeWaitForSingleObject(WakeUpThread, timeout = 2s 或 无限)
        ├── 事件唤醒：inflate/deflate 处理 + 低内存时即时 Step
        └── 2s 超时：BalloonReportStep（周期主体）

BalloonReportLowMemWatchRoutine（新增专用线程，仅 FPR 激活时）
  └── KeWaitForMultipleObjects(evLowMem | WatchStopEvent)
        └── LowMemoryCondition 置位 → KeSetEvent(WakeUpThread) 即时唤醒 worker
        └── 事件持续 signaled 期间 1s 间隔复查（NotificationEvent 防忙等）
```

生命周期：D0Entry 创建（evLowMem 之后）→ D0ExitPre/D0Exit 先于 evLowMem 句柄
关闭和 worker 停止而停止。watch 创建失败降级为纯轮询（worker 的周期检查是兜底）。

单线程原则：held MDL 链与统计只被 worker 访问（归还路径在 worker 停止后执行），
无需锁；reporting_vq 与 inf/def 队列共用 InfDefQueueLock。

## 3. 三个核心机制（触发 / 节奏 / 地址）

### 3.1 什么时候上报：周期轮询 + 水位闸门

Windows 不给驱动 free list 可见性（无 Linux 的 page-release 钩子），因此采用纯周期模型：

```
每 2s（REPORTING_INTERVAL_MS，对齐 Linux page_reporting_delay_ms 量级）：
  ① IsLowMemory → 全量归还，return
  ② commit 余量 < max(CommitLimit/10, 128MB) → 渐进归还一半，return
  ③ available < 水位/2 → 渐进归还一半，return
  ④ available ≤ 水位 → 静默等待
  ⑤ 否则 → 分配并上报
水位 = MinFreeMb 覆盖值，或 auto = max(RAM/8, 256MB)
```

首次上报：DRIVER_OK 后第一个周期。低内存响应：watch 线程即时（≤ 周期）。

### 3.2 每次上报多久：三层节奏

| 层 | 粒度 | 控制 |
|---|---|---|
| vq 请求 | 32 段 = 64MB（= QEMU vring 深度） | add_buf + kick + 同步 ack（1s 超时兜底） |
| 周期 | ≤ 8 批 = 512MB（REPORTING_BATCHES_PER_CYCLE） | 批间复查水位/commit 即时刹车 |
| 全局收敛 | 4GB guest park 2.6GB 实测 10~20s | 分配失败（2M 块耗尽/内存压力）即停 |

稳态开销：每周期一次 ZwQuerySystemInformation + 比较，近似为零。

### 3.3 从哪里开始上报：由 MM 决定（设计上不控制）

分配调用为 `MmAllocatePagesForMdlEx(0, MAX, SkipBytes=2MB, 64MB,
REQUIRE_CONTIGUOUS_CHUNKS | MM_DONT_ZERO_ALLOCATION)`：

- **没有地址起点/游标**。SkipBytes 在 REQUIRE_CONTIGUOUS_CHUNKS 模式下是块粒度约束
  （每块恰好 2MB 且 2MB 对齐），不是起点。
- **供给顺序（MSDN 明文）**：大页缓存优先；耗尽后由 MM 构造新的 2MB 连续块
  （"attempts to construct additional large pages, which may take a long time"）。
  构造过程的原料链（从 zeroed/free/standby 哪条链取料）无公开文档，不做假设。
  实测块物理聚簇（trace 连续递减 2M）是 MM 行为而非设计保证。
- 驱动只做验收：512 连续 PFN + 首 PFN 2M 对齐才上报；不合格块跳过但仍 hold。
- 同地址二次上报是正常行为（归还 → 回 MM → 再分配 → 再上报），对 host 幂等。

两个 flag 的选择理由：

- `MM_ALLOCATE_REQUIRE_CONTIGUOUS_CHUNKS`：获得“2MB 大小 + 2MB 边界对齐”块的唯一
  官方途径；部分分配时每块仍保证完整 2MB，不浪费。相邻但未采用的选项：
  `MM_ALLOCATE_PREFER_CONTIGUOUS`（尽力而为，不满足硬约束）；`MM_ALLOCATE_FAST_LARGE_PAGES`
  （仅从大页缓存取、缓存空即失败——不采用：接受构造延迟换取更大 park 上限）。
- `MM_DONT_ZERO_ALLOCATION`：清零 = 触碰每页 = host 侧 fault in + dirty，与 FPR
  目的（页保持 clean/discardable）直接冲突。文档条件：不暴露给用户态即满足
  （我们的页无 VA、不映射）。信息泄露无新增面：guest 内核本为特权，host 本可读
  全部 guest 内存；归还后 MM 按需 zero。

与 Linux 的差异：Linux 有 zone/order/migratetype 游标与 PageReported 标记（能
"扫描" free list）；我们只能"申请"，"完成"由水位与 2M 块可获得性（碎片化 floor）
定义。host 不关心顺序与完整性，只关心段集合的 2M 对齐。

## 4. 保护机制矩阵

| 触发 | 条件 | 动作 | 响应延迟 |
|---|---|---|---|
| LowMemoryCondition | MM 置位（阈值内核黑盒，不可配置） | 全量归还 | 即时（watch 线程） |
| commit 余量低 | < max(CommitLimit/10, 128MB) | 渐进归还一半/周期 | ≤ 2s |
| 物理水位低 | available < 水位/2 | 渐进归还一半/周期 | ≤ 2s |
| 分配失败 | MM 拒绝（压力/块耗尽） | 本周期停止 | 即时 |

commit 语义要点：held 页 pinned + committed，双占物理与记账；commit 是 demand-zero
延迟分配，承诺不占物理，故物理水位对 commit 余量无蕴含关系（曾误判"无 pagefile 时
蕴含成立"，被推翻——见 febfb2e2）。system managed pagefile 自动扩容可吸收大部分
commit 压力。

## 5. 可配置参数（注册表，INF 预置默认值）

```
HKLM\SYSTEM\CurrentControlSet\Services\BALLOON\Parameters
  EnableFpr  (DWORD, 默认 1)：0 = 不协商 F_PAGE_REPORTING（逃生门）
  MinFreeMb  (DWORD, 默认 0 = auto)：保留可用内存，钳制 [64MB, RAM/2]
```

机制部分（LowMemoryCondition、迟滞带、渐进归还节奏）刻意不开放。

## 6. 异常处理

- 可预期路径：surprise removal / D0 exit / reboot → 停线程 → ReleaseAll 归还；
  host 不 ack → 页保持 hold，不重发，等恢复或移除。
- 不可预期路径：guest bugcheck → 重启自愈；已上报页由 host demand paging 兜底
  （RAMBlock 地址空间常在，discard 只释放物理后备，任何写入按需 fault 回来，
  host 零动作）。崩溃转储中这些页为零（本就是空闲页，取证价值低）。
- 驱动自身 bug（链表/计数损坏）：无结构保护，靠 Driver Verifier + review
  （测试矩阵 M.1）。
- 数据安全基座：上报页从分配到归还全程不承载任何数据（DONT_ZERO + 不触碰 +
  归还后按 zeroed 语义供给），任何一侧异常的最坏结果是内存量错误，不是数据损坏。

## 7. 实测数据摘要（4GB guest / Win Server 2022 / QEMU 9.2）

- park：RSS 4.16GB → 1.4GB（释放 ~2.6GB），hold 稳定性 7 样本反弹 0~1MB
- 2M 对齐：ram_block_discard_range 全样本 0x200000 且地址 2M 对齐（零例外）
- 压力归还：guest 吃 1.5GB，RSS +1.7GB，首个 2s 采样内启动
- re-park：压力释放后 ~10s 回基线；qemu-cpu 无可观测变化
- commit：park 2.6GB 与 committed bytes 一比一（3852MB/4607MB limit，fixed 512MB
  pagefile）；headroom 高于阈值时保护静默、行为与无保护版本一致
- setmem 回归：inflate/deflate 正常（低内存保护与 FPR 归还协同），deflate 精确恢复

## 8. 已知边界

- IOMMU（ACCESS_PLATFORM）下不协商 FPR：上报页无 VA，无法 MapTransfer 取 IOVA
- 2M 块碎片化 floor：available ~500MB 处 park 耗尽可分配块（与水位无关的自然极限）
- commit 触发态的运行时快照未取得（guest 侧大预留施压受 PS 分配限制）；
  触发路径与物理水位共享已验证的渐进归还代码
- x86/ARM64 编译依赖上游 CI（本机 WDK 26100 无相应工具链）
- **REPORTING_MAX_SEGMENTS=32 为硬编码**（对齐 QEMU reporting_vq 的深度 32，非从
  vring 读取——virtio-win 库缺少 Linux 的 `virtqueue_get_vring_size()` 对应接口；
  vring 深度存于库内部结构 `vq->vring.num`，加访问器是候选的上游小改动）。
  兜底：批量 add_buf 失败自动降级为逐段请求（功能正确、吞吐降级）。另注意
  VirtIOPCIModern 在 ring 内存分配失败时会把深度折半重试，实际深度可能小于设备
  offer 值，此时依赖同一降级路径。
