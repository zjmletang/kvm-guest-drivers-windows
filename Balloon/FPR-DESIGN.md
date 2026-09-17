# Balloon Free Page Reporting (FPR) 设计文档

> 对应实现：`Balloon/sys/reporting.c`（版本 58707 起）
> 系列提交：210aaf80 → 50fe3ebd → 74f4ab2c → 22ad5876 → febfb2e2 → 09043d2b →
> ab013f9a → e30e472f → 94bf0ac3 → cb84b824 →（cast/文档各一笔）
> 配套文档：`FPR-COMMIT-MODEL.md`（Windows 双账本模型与 commit 保护推导）、
> `FPR-DEMO.md`（演示手册与实测数据）、`tests/fpr/fpr-demo.cast`（51 秒实况录像）

## 0. 代码地图（快速导航）

| 位置 | 职责 |
|---|---|
| `reporting.c: BalloonReportStep` | 周期主体：三态查询 → 释放分支（低内存/commit/水位）→ 冷却检查 → park 循环（每 batch 复查） |
| `reporting.c: ReportingQueryMemoryState` | 一次 ZwQuerySystemInformation 取 available/commitLimit/committed |
| `reporting.c: ReportingCommitHeadroomLow` | commit 阈值判定：MinCommitMb 覆盖或 max(RAM/10, 128MB) |
| `reporting.c: ReportingStartCooldown` | 全量释放后的冷却：60s 起跳，重复触发翻倍至 5min，10min 安静后重置 |
| `reporting.c: ReportingReadParameters` | 统一读取 5 个注册表参数（MinFreeMb/MinCommitMb/ReportIntervalMs/CooldownSec…） |
| `reporting.c: BalloonReportLowMemWatchRoutine` | watch 线程：LowMemoryCondition 事件（毫秒级）+ commit 100ms 轮询快路径 |
| `reporting.c: BalloonReportInitialize` | 参数读取、RAM 缓存（ReportingTotalPages）、vring 深度读取（min(32, vring_size)） |
| `Device.c: BalloonRoutine` | worker 主循环：超时 = ReportIntervalMs → Step；**事件唤醒无条件 Step**（cb84b824 修复） |
| `ProtoTypes.h` | 全部宏（阈值/冷却/批次）与 DEVICE_CONTEXT 字段 |

## 1. 目标与方案

在 Windows guest 上实现 `VIRTIO_BALLOON_F_PAGE_REPORTING`（virtio 1.x §5.5.6.7）：

- **report-then-hold**：上报后的页保持驱动持有（不归还 OS），guest 无法触碰，host 侧
  discard 效果持续有效；guest 需要内存时才归还（水位/低内存/commit 触发）。
  依据：report-then-release 在 Windows 不可行（归还页被 MM zeroing 触碰，host 侧
  回收效果立即消失）；hold 语义已由 virtio-comment 讨论确认在现有 spec 之内，
  无需新 feature bit。
- **2MB 对齐上报**（大小与边界）：对齐 host 侧 THP 回收粒度与 Linux pageblock_order
  默认粒度；spec "SHOULD attempt to report large pages rather than smaller ones"。

## 2. 线程模型

```
BalloonRoutine（复用现有 balloon worker 线程）
  └── KeWaitForSingleObject(WakeUpThread, timeout = ReportIntervalMs 或 无限)
        ├── 事件唤醒：inflate/deflate 处理 + 无条件 BalloonReportStep
        │   （Step 自带全部前置检查，误唤醒无害——cb84b824 修复：
        │    此前仅 IsLowMemory 时才 Step，watch 的 commit 唤醒被忽略）
        └── 超时：BalloonReportStep（周期主体）

BalloonReportLowMemWatchRoutine（专用线程，仅 FPR 激活时）
  └── KeWaitForMultipleObjects(evLowMem | WatchStopEvent, timeout = 100ms)
        ├── WAIT_0（物理事件）：即时唤醒 worker；置位期间 1s 复查防忙等
        └── TIMEOUT + held>0：commit 快路径——每 100ms 查承诺余量，
            低则唤醒 worker（Windows 无 "commit 低" 内核事件，只能轮询；
            held=0 时跳过查询，零开销）
```

生命周期：D0Entry 创建（evLowMem 之后）→ D0Exit 先于句柄关闭而停止。
watch 创建失败降级为纯轮询。单线程原则：held MDL 链与统计只被 worker 访问，无锁。

## 3. 三个核心问题（触发 / 节奏 / 地址）

### 3.1 什么时候上报：周期轮询 + 四道闸门 + 冷却

Windows 不给驱动 free list 可见性，采用纯周期模型：

```
每 ReportIntervalMs（默认 2000，对齐 Linux page_reporting_delay_ms）：
  ① IsLowMemory → 全量归还 + 进入冷却，return
  ② commit 余量 < max(RAM/10, 128MB) 或 MinCommitMb 覆盖 → 渐进归还一半，return
  ③ available < 水位/2 → 渐进归还一半，return
  ④ available ≤ 水位 → 静默等待
  ⑤ 冷却期内（全量释放后 60s 起，指数退避）→ 跳过 park
  ⑥ 否则 → 分配并上报（每 batch 复查 ②③④）
水位 = MinFreeMb 覆盖，或 auto = max(RAM/8, 256MB)
```

**响应模型**（连续分配场景的语义）：
- 单次 < MinCommitMb 的分配：稳态下必成功；
- 突发速率 ≤ 预留/0.1s（默认值下 ≥ 4GB/s）：全程无失败（watch 100ms 快路径
  实测：10×128MB 每 500ms 全部成功，释放以 100ms 级跟上 256MB/s 消耗）；
- 更高速率：响应窗口内的部分 1455 失败后恢复——Windows 无 commit 同步回调，
  硬保证不可实现（Linux watermark 同理）。

### 3.2 每次上报多久：三层节奏

| 层 | 粒度 | 控制 |
|---|---|---|
| vq 请求 | min(32, vring_size) 段 = ≤64MB | add_buf + kick + 同步 ack（1s 超时兜底） |
| 周期 | ≤ 8 批 = ≤512MB | 批间复查水位/commit 即时刹车 |
| 全局收敛 | 4GB park 2.6GB ~15s；16GB park 12.7GB ~1min | 分配失败即停 |

深度来源：`virtqueue_get_vring_size(RepVirtQueue)`（virtio-win 库新增接口，
上游分支 vring-size / PR #1647），上限取栈上数组界 32。QEMU 的 reporting_vq
深度硬编码 32，ring 内存不足时传输层可折半——动态读取天然适配。

### 3.3 从哪里开始上报：由 MM 决定（设计上不控制）

分配调用为 `MmAllocatePagesForMdlEx(0, MAX, SkipBytes=2MB, 批量,
REQUIRE_CONTIGUOUS_CHUNKS | MM_DONT_ZERO_ALLOCATION)`：

- SkipBytes 在 REQUIRE_CONTIGUOUS_CHUNKS 模式下是块粒度约束（每块恰好 2MB
  且 2MB 对齐），不是起点；供给顺序（MSDN 明文）大页缓存优先，耗尽后构造。
- 驱动只做验收；同地址二次上报是正常行为（归还 → 回 MM → 再分配），对 host 幂等。
- `MM_DONT_ZERO_ALLOCATION`：清零 = 触碰 = host 侧 fault in，与 FPR 目的直接冲突。
- 与 Linux 的差异：Linux 有 free list 游标与 PageReported 标记（能"扫描"）；
  我们只能"申请"，"完成"由水位与 2M 块可获得性定义。

## 4. 保护机制矩阵（响应延迟实测）

| 触发 | 条件 | 动作 | 响应延迟 |
|---|---|---|---|
| LowMemoryCondition | MM 置位（实测 ~7-11% RAM，黑盒） | 全量归还 + 冷却 | **毫秒级**（事件） |
| commit 余量低 | < max(RAM/10, 128MB) 或 MinCommitMb | 渐进归还一半/周期 | **≤100ms**（watch 轮询） |
| 物理水位低 | available < 水位/2 | 渐进归还一半/周期 | ≤ 周期 |
| 分配失败 | MM 拒绝 | 本周期停止 | 即时 |

commit 语义要点（详见 FPR-COMMIT-MODEL.md）：held 页 pinned+committed 双占；
demand-zero 使两账独立（无 pagefile 亦不蕴含——曾被反例推翻）；阈值锚定 RAM
而非 CommitLimit（大 pagefile 膨胀分母会导致过度保守/振荡，Case 实测）；
两本账的双向不对称各有守护者：水位线看不见的（未物化承诺）commit 检查看见，
commit 看不见的（文件后备页）水位线看见。

## 5. 可配置参数（注册表，INF 预置默认值）

```
HKLM\SYSTEM\CurrentControlSet\Services\BALLOON\Parameters
  EnableFpr       (DWORD, 默认 1)      ：0 = 不协商 F_PAGE_REPORTING（逃生门）
  MinFreeMb       (DWORD, 默认 0=auto) ：物理水位覆盖，钳制 [64MB, RAM/2]
  MinCommitMb     (DWORD, 默认 0=auto) ：承诺预留覆盖，钳制 [128MB, RAM/2]
  ReportIntervalMs(DWORD, 默认 0=2000)  ：周期，钳制 [100, 60000]
  CooldownSec     (DWORD, 默认 0=60)   ：冷却起跳值，钳制 [1, 600]
```

机制部分（迟滞带、指数退避、渐进归还节奏）刻意不开放。MinCommitMb 的动机：
2GB + 冷启动 pagefile=0 场景 park 挤占承诺余量，大承诺分配 1455（见 §8）。

## 6. 异常处理

- 可预期路径：surprise removal / D0 exit / reboot → 停线程 → ReleaseAll 归还；
  host 不 ack → 页保持 hold，不重发，等恢复或移除。
- 不可预期路径：guest bugcheck → 重启自愈；host 侧 demand paging 兜底。
- 驱动自身 bug：无结构保护，靠 Driver Verifier + review（测试矩阵 M.1，未跑）。
- 数据安全基座：上报页从分配到归还全程不承载数据，最坏结果是内存量错误。

## 7. 实测数据摘要（4GB Win11 / QEMU 9.2，58707）

- **park**：RSS 4.15GB → 1.4GB（释放 ~2.6GB）；16GB 配置 park 12.7GB ~1min 收敛；
  无 pagefile 时 park 浅 ~200MB（commit 上限先绑，模型实证）
- **2M 对齐**：5480 条 ram_block_discard_range 全部 0x200000 且 2M 对齐（零例外）
- **压力归还**：2800MB 压力首秒即响应（RSS t+1s +947MB），3~9s 全量完成；
  压力超 available 缓冲 7.6 倍零失败；释放后 10~35s 完全 re-park
- **冷却**：全量释放后 5s 真空期（CooldownSec=5 实验）；重复触发翻倍；
  压力停后 ~10s 恢复
- **commit 保护**：脉冲 10×128MB 全部成功（headroom 锯齿 + 释放跳升）；
  大 pagefile（16GB）低余量下旧驱动振荡（RSS 每分钟 ±1GB）vs 新驱动 4 分钟零方差；
  无 pagefile 施压穿透阈值 → 1 秒内释放一半（watch 修复的正面验证）
- **watch 修复（cb84b824）**：trace 显示修复前 watch 每 100ms 唤醒被 worker 忽略
  （worker 只认 IsLowMemory）；修复后唤醒路径无条件 Step

## 8. 已知边界

- IOMMU（ACCESS_PLATFORM）下不协商 FPR：上报页无 VA，无法 MapTransfer 取 IOVA
- 2M 块碎片化 floor：available ~500MB 处 park 耗尽可分配块（与水位无关的自然极限）
- **2GB + 冷启动 pagefile=0**：park ~800MB 后承诺余量 ~260MB，应用大承诺分配
  1455 失败；自愈路径 = 自动 pagefile 增长后重试成功；缓解 = MinCommitMb 调大预留
- x86/ARM64 编译依赖上游 CI（本机 WDK 26100 无相应工具链）
- Driver Verifier（测试矩阵 M.1）未跑
- ~~REPORTING_MAX_SEGMENTS=32 硬编码~~：已解决（virtqueue_get_vring_size，
  vring-size 分支 / PR #1647）

## 9. 当前状态（供接手的 agent/人）

- **分支**：`fpr`（已 push 到 fork，14 笔）；`vring-size` 分支独立提上游
  （PR #1647 评审中，含 vioserial clang-format CI 修复）
- **上游拆分**：vring-size PR 合入后，fpr rebase 到新 master 并 drop 重复的
  `[VirtIO]` commit（689e1f67），然后 FPR 系列开 PR
- **文档语言**：三份 .md 均为中文（审阅定稿后需英文化随 PR）
- **测试资产**：`tests/fpr/run-fpr-tests.ps1`（T1/T2.x 自动矩阵）；
  `tests/fpr/fpr-demo.cast`（asciinema，`asciinema play` 播放）；
  测试工具（eatmem/lowmem-watch/commit-reserve，_tools/ 目录未入库）
- **实验环境**：裸金属 47.83.225.42 的 fpr_upstream VM（4GB/自动 pagefile/
  58707），WinRM 经 winrm_ps.py；施压用 schtasks（WinRM 会话会杀 Start-Process
  子进程）；WPP trace 用 logman level 255 + tracepdb/tracefmt 解码
