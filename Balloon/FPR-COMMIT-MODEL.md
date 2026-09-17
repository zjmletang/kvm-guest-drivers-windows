# Windows 内存的双账本模型与 FPR 的 commit 保护

本文档梳理 Windows 内存管理的"两本账"模型，推导为什么 Free Page Reporting (FPR)
的 hold 方案必须同时保护物理账（available）和承诺账（commit），并与 Linux 对比。

## 1. 两本账总览

|  | 物理账 | 承诺账 |
|---|---|---|
| 度量 | `AvailablePages`（= free + zeroed + standby） | `CommittedPages` / `CommitLimit` |
| 语义 | **现在就能拿去用的物理页** | **所有已承诺、尚未释放的后备存储义务** |
| 上限 | RAM | RAM + Σ pagefile 当前容量 |
| 何时消耗 | 真实占用物理页的一切 | 任何承诺（哪怕从不访问） |

两本账通过 **demand-zero 延迟分配** 解耦：

```
VirtualAlloc(MEM_COMMIT)  →  只在承诺账记一笔（CommittedPages++）
                              不占任何物理页
首次访问该页              →  缺页 → MM 取一物理页清零物化
                              （物理账减少，承诺账不变）
```

推论：**一个系统可以承诺账几乎满、物理账几乎空**（大量"reserve 了不用"的
工作负载：Java 预留堆、数据库 buffer reserve、预留型 VirtualAlloc）；反之亦然。

## 2. CommitLimit 与消耗者

`CommitLimit = RAM + 所有 pagefile 的当前容量`。

- 无 pagefile：CommitLimit = RAM（如 4GB VM 就是 4GB）；
- 固定 pagefile：RAM + 固定容量；
- 系统托管 pagefile：可随压力增长（但受磁盘空间/上限约束，**不是无限**）。

承诺账的消耗者（全部计入 CommittedPages）：
- 用户态：`VirtualAlloc(MEM_COMMIT)`（不 touch 也算）、私有内存映射；
- 内核态：非分页/分页池、页表等内核结构；
- **FPR park 的页**：`MmAllocatePagesForMdlEx` 分配的 MDL 页是"立即物化的
  承诺"——既占物理（pinned），又计承诺。

> 实测证据（fixed 512MB pagefile，4GB VM）：park 2.6GB 期间 CommittedPages
> 从 ~1250MB 升至 3852MB，增量 +2.6GB 与 park 量**精确相等**——证明 MDL
> 页逐页计入承诺账。

## 3. FPR 与两本账的关系

park 一页（`MmAllocatePagesForMdlEx`）：物理账 −1（available 减少）、
承诺账 +1（CommittedPages 增加）。

释放一页（`MmFreePagesFromMdl`）：物理账 +1、承诺账 −1（两账同时归还）。

FPR 的三道物理侧检查（水位线、水位线/2 渐进释放、LowMemoryCondition 全量
释放）只看物理账。**承诺账需要一个独立的检查**——原因见下节推导。

## 4. 为什么水位线保护不了承诺账——先到上限的推导

设（无 pagefile 情形，CommitLimit = RAM = R）：

- `P` = 已物化承诺（真实占物理：进程工作集、内核、以及 park 的页）
- `U` = 未物化承诺（demand-zero 待物化部分），`CommittedPages = P + U`
- `A` = available ≈ R − P（简化：物理页要么在使用要么在可用）
- `W` = 水位线（默认 max(RAM/8, 256MB)）

park 停止的两个上限：

```
物理上限（水位线）：X_phys = A − W   = R − P − W
承诺上限（承诺余量）：X_commit        = R − (P + U)

X_commit < X_phys   ⟺   U > W
```

**结论：只要未物化承诺超过水位线，承诺上限先绑**——park 在物理还很充裕时
就把承诺打满。

### 反例复现（即当初推翻"蕴含关系"的用例）

4GB 无 pagefile VM：CommittedPages 3.9GB（其中 U ≈ 3.9GB 未物化），
available ≈ 4GB（几乎无人 touch 物理）。

```
X_phys   = 4G − 0.1G − 0.5G = 3.4GB   （物理水位线允许 park 3.4GB）
X_commit = 4G − 3.9G        = 0.1GB   （承诺余量只允许 park 0.1GB！）
```

物理侧一切正常（available 4GB >> 水位线 512MB），但 park 100MB 后承诺就满了。

### 没有独立检查时的实际行为

park 会一直进行到 `MmAllocatePagesForMdlEx` **因承诺耗尽而失败**（MM 在
CommittedPages 逼近 CommitLimit 时硬拒分配，返回 NULL/空 MDL）。此时：

- 承诺余量 ≈ 0，系统处于"承诺饥饿"：任何进程的 `VirtualAlloc`（哪怕一页）、
  进程启动、线程栈扩展、驱动池增长全部失败；
- **而物理账上 available 依然充裕，水位线和 LowMemoryCondition 都毫无感知**
  （LowMemoryCondition 是物理内存事件，不反映承诺余量）。

FPR 的 commit 检查就是把这个止损点从“MM 硬失败（余量≈0）”提前到
**余量 < max(RAM/10, 128MB)**：触发后释放一半持有页（指数收敛，
2–4 秒出清），应用的承诺分配随之恢复。

### 4.1 双向不对称与守护者互补

两本账的不对称有两个方向，各有一个守护者，无盲区：

| 不对称方向 | 典型来源 | 守护者 | 防线 |
|---|---|---|---|
| **承诺紧、物理松**（U > W，上述推导） | reserve 不 touch 的工作负载 | commit 检查 | Step 入口 + batch 复查 |
| **物理紧、承诺松** | 文件脏页积压（有文件后备、不计承诺）/大页 | 物理水位线 | Step 入口 + batch 复查 + MM 分配失败兑底 |

后一方向的“占物理不计承诺”来源：文件缓存/脏页的内容可从文件重读
（文件即后备，不需 pagefile 承诺）；大页锁定物理永不换出。此方向下
FPR 的行为：水位线在 park 前置检查拦住（available ≤ W 不 park）；
TOCTOU 窗口内由 batch 复查；极端时序下 MmAllocatePagesForMdlEx
本身从 MM 空闲供给拿页，拿不到返回 NULL（非抢占式），Step break。

两个方向的守护者恰好互补：**水位线看不见的（未物化承诺），commit
检查看见；commit 检查看不见的（文件后备页），水位线看见**；同时吃
两本账的部分（工作集、内核池、MDL 页）任一账紧都触发保护。

## 5. 保护设计汇总

| 触发（Step 每轮检查 + 每 batch 复查） | 动作 | 恢复的账本 |
|---|---|---|
| LowMemoryCondition 置位（watch 线程即时唤醒） | 全量释放 | 两账 |
| 承诺余量 < max(**RAM**/10, 128MB) | 释放一半/轮 | 承诺（同时物理） |
| available < 水位线/2 | 释放一半/轮 | 物理（同时承诺） |
| available ≤ 水位线 | 停止 park | —— |

内置不可配置（理由：承诺耗尽的后果是系统级分配失败，不应留给部署方决策；
MinFreeMb 只开放物理水位线）。系统托管 pagefile 下承诺压力大多被 pagefile
增长吸收，此检查通常静默（实测：托管模式 headroom 1493MB > 阈值 511MB，静默）。

**阈值锚定 RAM 而非 CommitLimit 的理由**：CommitLimit 随 pagefile 膨胀，但
大 pagefile 恰恰是承诺余量低最无害的配置（物化可以页出到 pagefile，物理侧
有水位线兜底）——预留量没有理由随 pagefile 增长。若以 CommitLimit 为分母，
4GB RAM + 32GB pagefile 的系统阈值会膨胀到 3.6GB，导致大量承诺场景下
FPR 完全停止工作（尽管物理充裕）；锚定 RAM 后阈值为 422MB，行为与无
pagefile 系统一致，且对 pagefile ≤ RAM 的配置零影响。

## 6. 与 Linux 的对比

| 维度 | Windows | Linux（默认） |
|---|---|---|
| 承诺模型 | CommitLimit 硬上限，逐承诺记账 | overcommit（mode 0/1 无硬上限） |
| 虚拟分配 | VirtualAlloc 立即记账 | mmap/brk 默认不查总量 |
| 物化 | demand-zero 缺页物化 | 缺页分配（语义相同） |
| 内核/FPR 页记账 | **计入 CommittedPages**（实测） | **不经过用户态 overcommit 记账**（`__vm_enough_memory` 只管 mmap/brk 路径） |
| FPR 页来源 | MmAllocatePagesForMdlEx（立即物化+承诺） | buddy 空闲页 isolate，report 后**立即归还**（不 hold） |

三点差异决定了"Linux FPR 无需 commit 检查、Windows hold 方案必须有"：

1. Linux 默认无承诺硬上限（严格模式 mode 2 存在 CommitLimit 类似物，但也
   只作用于用户态路径）；
2. Linux 内核持有页不走承诺记账，Windows MDL 页走；
3. Linux FPR 不 hold（report 完即还 buddy），不存在长期占用。

## 7. 实测证据索引

| 场景 | 观测 | 结论 |
|---|---|---|
| **开关对照实验（系统托管 pagefile，最干净）**：park 稳态 vs EnableFpr=0 全释放 vs 恢复 park，三态对比 | park 2879MB 时 CommittedBytes 3879 vs 释放后 1072（差 2807，误差 2.5%）；复现轮 park 2684MB 时差 2696（**误差仅 12MB**） | **MDL 页逐页计入承诺账，释放后精确回落；双向可重复** |
| fixed 512MB pagefile，park 2.6GB | CommittedPages 1250→3852MB（+2.6GB 精确对应） | 与上条互证 |
| 系统托管 pagefile，park 稳态 | headroom 1493MB > 阈值 511MB | 托管模式检查静默 |
| fixed 128MB pagefile + MinFreeMb=64（极限配置） | headroom 735MB > 阈值 422MB，系统稳定 | 极限配置下检查正确静默 |
| 承诺触发态运行时快照 | guest 侧施压受限（PowerShell job OOM / 独立进程分配失败），未取得 | 已知边界：阈值算术（4223/10=422）与已验证的物理触发共享同一释放代码路径 |

注：MSDN 对 MmAllocatePagesForMdlEx 的 Remarks 未提及 commit charge，
上述实测是该问题的第一手资料。

## 8. 与 FPR-DESIGN.md 的关系

FPR-DESIGN.md 描述整体设计与三道触发；本文档是承诺账部分的深入展开
（模型、推导、对比），供审阅模型之用。
