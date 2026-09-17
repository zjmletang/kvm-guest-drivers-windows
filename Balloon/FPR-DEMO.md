# Free Page Reporting 演示手册

> 目标：用一个下午的可复现实验，回答两个问题——**这功能有多大价值**（host
> 真实拿回内存）、**它有多安全**（guest 需要时还得快、永远不会压死系统）。
> 所有数据来自 4GB RAM 的 Windows 11 x64 虚拟机（QEMU virtio-balloon）。

## Demo 1：核心价值——host 拿回空闲内存

**操作**：同一 guest，`EnableFpr` 注册表参数 0/1 切换后重启，观察 QEMU
进程 RSS（host 侧真实驻留）。

| 配置 | guest 视角 | QEMU 进程 RSS |
|---|---|---|
| FPR off | 一切正常 | **4151 MB** |
| FPR on | 一切正常 | **1272 MB** |

**解读**：guest 空闲约 2.8GB，FPR 让 host 真实收回等量内存——这是
free page reporting 的全部价值主张，一行表格说清。

## Demo 2：安全性核心——guest 要内存时还得快

**操作**：park 稳态下，guest 内运行 memstress 分配并触碰 1500MB；
随后释放。

| 时间线 | 事件 | host RSS |
|---|---|---|
| t=0 | memstress 启动（吃 1.5GB） | 1492 MB |
| t≈2s | FPR 归还启动，压力分配成功 | **+1728 MB** |
| t=60s | 压力释放 | 回落中 |
| t≈70s | 自动 re-park 完成 | 1530 MB（回到基线） |

**解读**：hold 方案不伤害 guest——压力分配 2 秒内得到供给，压力消失后
自动恢复上报。（自动化复现：测试套件 T2.7/T2.8。）

## Demo 3：上报质量——2MB 对齐，host 侧 THP 友好

**操作**：QEMU 开启 `ram_block_discard_range` trace，运行完整测试
周期后分析日志。

```
discard 记录总数：5480 条（FPR 路径）
  大小：100% = 0x200000（2MB）
  边界：100% 2MB 对齐
  违例：0
（另有 5029 条 4KB 记录来自传统 balloon inflate，时间窗完全分离）
```

**解读**：host 收到的是 THP 粒度的整页，无碎片。（自动化复现：T2.4。）

## Demo 4：工作原理透明——WPP trace 决策流

**操作**：guest 内 logman 抓 WPP trace（level 255），tracefmt 解码。

```
State: available 768K pages, watermark 131K pages, commit headroom 1160K of 1310K pages, 0 pages held
Batch 0: 32 blocks, 32 pending segments, 65536 pages held
Batch 1: 32 blocks, 64 pending segments, 131072 pages held
...
Held 262144 pages, reported 262144 pages in total
（压力期）
LowMemoryCondition set, waking the worker
Low memory condition, releasing 262144 pages
LowMemoryCondition cleared
State: available 700K pages, watermark 131K pages, ... , 0 pages held
```

**解读**：每 2 秒一次的决策输入（available/水位线/承诺余量/持有量）、
每个 batch 的分配量、低内存事件的即时响应，全部可见。

## Demo 5：commit 安全网——精细且不失灵

**背景**：FPR 持有的每页同时占用 Windows 的承诺账（实测：park 2.6GB
→ 系统承诺精确 +2.6GB，释放后逐页归还，误差 12MB）。承诺余量保护
阈值 = max(RAM/10, 128MB)。

**操作**：固定 8GB pagefile（CommitLimit 12GB），用 commit-reserve.exe
（VirtualAlloc 不触碰，制造纯承诺压力）施压 8.2GB，对比新旧驱动。

| 场景 | 驱动 | 行为 | host RSS |
|---|---|---|---|
| 同等压力 8.2GB | 旧（阈值=CommitLimit/10=1.2GB） | **振荡**：park↔release 每分钟 ±1GB | 2099→3058→2034→3058 |
| 同等压力 8.2GB | 新（阈值=RAM/10=410MB） | **稳定 park** | **1390-1391（4 分钟零方差）** |
| 追加 400MB（余量压至阈值附近） | 新 | 平滑收缩 park 深度 ~127MB，无振荡 | 1391→1518 |
| 恢复默认配置 | 新 | 零回归 | 1392 稳态 |

**解读**：三重含义——① 保护真实存在（余量压破阈值时 park 深度自动让路）；
② 修复了阈值随 pagefile 膨胀导致的过度保守（振荡消失）；③ 默认配置
行为不变。承诺余量被挤占的场景正是"数据库/Java 预留大堆未触碰"的
典型负载——保护留住了应用的承诺空间，FPR 让路。

## 复现指引

| Demo | 方式 |
|---|---|
| 1/2/3 | `Balloon/tests/fpr/run-fpr-tests.ps1`（T2.2 / T2.7+T2.8 / T2.4），一键自动 |
| 4 | logman（WPP GUID 08cb9471-36fb-46ee-998b-d1bfbe1c4899，level 255）→ tracepdb 生成 TMF → tracefmt 解码 |
| 5 | commit-reserve.exe（随测试包）+ 固定 pagefile + 上述施压序列（完整脚本可提供） |
| 全部 | **asciinema 录像：`tests/fpr/fpr-demo.cast`**（51 秒，`asciinema play fpr-demo.cast` 播放） |

## 补充验证数据（响应模型实测）

### 压力响应精度（1 秒采样，EXP-LOWMEM）

```
t=0    RSS 1452（available 缓冲仅 367MB）
t=1s   RSS 2399  ← 首秒即响应（watch 事件驱动，非周期）
t=9s   RSS 4136  ← 全量释放完成
hold   RSS 3948 稳定（2800MB 压力全部成功，超 available 缓冲 7.6 倍零失败）
释放后 ~35s 完全 re-park
```

### 冷却期行为（CooldownSec=5，触发全释放后）

```
t=1s   全量释放（LowMemoryCondition，1 秒内）
t=2-6  冷却真空期：available 高达 2.7GB 但 park 不抢回
t=7    冷却过期后 park 尝试回归，被持续压力再压回（冷却翻倍 5→10s）
压力停止后 ~10s：完全 re-park
```

### 连续承诺分配的响应（修复后脉冲实验：10×128MB，500ms 间隔）

```
headroom：1440→1295→1161→1022→885→747→610→997→860→721→583
                                        ↑ 跌破 410MB 阈值 → watch 100ms 内
                                          唤醒 → 释放一半 +1.3GB
结果：10/10 全部成功（alive: 10），释放以 100ms 级节奏跟上 256MB/s 分配消耗
```

### 配置矩阵（同一 VM，切 pagefile/内存）

| 配置 | CommitLimit | 行为 |
|---|---|---|
| 4GB + 无 pagefile | 4095 | park ~2.5GB（比有 pagefile 浅 ~200MB，停在 commit 阈值线上——“U>W 时承诺上限先绑”的实证） |
| 4GB + 16GB pagefile | 20671 | 余量 870MB（410MB~3.6GB 区间）：旧驱动振荡，新驱动 park 稳定（RSS 4 分钟零方差） |
| 16GB + 自动 | 16383 | park 12.7GB，收敛 ~1 分钟，阈值自动适配 1.6GB |
| 2GB + 自动（冷启动） | 2047 | 下限分支实测（水位线 256MB floor、阈值 204MB）；注意：冷启动 pagefile=0 时大承诺分配会 1455，由 MinCommitMb 参数缓解 |

## 环境与驱动

- 裸金属宿主：QEMU 9.2，libvirt
- guest：Windows 11 x64，4GB RAM，virtio-balloon（reporting_vq 深度 32）
- 驱动：本分支 balloon.sys（feature bit 5 协商，WPP 观测打印内置）
