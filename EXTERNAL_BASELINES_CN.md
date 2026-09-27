# 外部强基线接入记录

## da Silva–Urrutia GVNS

本地已下载并在 Windows MinGW 环境下编译通过：

```text
external_baselines/TSPTW-master/bin/Run.exe
```

编译时使用 `-DWIN32` 并排除 Linux 专用的 `cputimer.cpp`，改用仓库中的 `windowstimer.cpp`。

## 重要的输入语义差异

该外部程序的 reader 读取的是：

```text
node_id x y demand ready_time due_time service_time
```

然后根据坐标重新构造欧氏旅行时间和服务时间；当前项目的 SPB 文件使用的是完整非对称旅行代价矩阵和时间窗。因此不能直接把当前矩阵文件喂给该程序，也不能默认外部目标值与当前 `EvaluateTSPTWRoute` 完全相同。

正式比较前需要先完成输入语义适配和路线回灌校验：

```text
外部程序输出路线
→ 当前EvaluateTSPTWRoute复算
→ 核对时间窗可行性和tour cost
```

当前GVNS只完成了编译和接口pilot，不作为正式强基线结论。


## Windows adapter

A local `adapter_main.cpp` was added to the ignored checkout and compiled as:

```text
external_baselines/TSPTW-master/bin/gvns_adapter.exe
```

The adapter accepts an instance path, iteration count, and seed, and prints cost, penalty, and route. A direct run on a 101-node Solomon file was intentionally stopped because the original implementation is computationally expensive at that size; this is a pilot/integration check, not a formal baseline result.
