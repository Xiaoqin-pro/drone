# PSO_UAV_TW_3D_Dynamic_Benchmark20

这是20任务动态三维无人机路径规划基准版本，用来比较有限迭代预算下的 Restart-PSO 和 Warm-start-PSO。

## 当前场景

- 20个初始客户；
- 自主程序化三维山地；
- 三维箱体障碍物；
- 时间窗和服务时间；
- 分层 Routing-Path 结构；
- 航段缓存复用；
- 动态订单取消和新增；
- Mild / Moderate / Severe 三种动态变化强度；
- 10、20、30、50代有限预算。

## 运行

```matlab
RunDynamicBenchmark
```

当前 `nRuns=1` 仅用于验证结构和指标链条。正式实验应把 `nRuns` 提高到至少30，并固定相同场景、随机种子集合和计算预算。

## 结果

保存在 `results`：

- `dynamic_benchmark20_summary.csv`：每个条件和每个动态事件的详细结果；
- `dynamic_benchmark20_means.csv`：分组汇总；
- `dynamic_benchmark20_result.mat`：完整实验数据。

记录的核心指标包括：

```text
response_time
warm_start_fe
routing_fe
total_fe
initial_best_fitness
final_fitness
total_late
obstacle_violation
first_feasible_fe
```

## 当前研究定位

本版本不是最终创新算法，而是用于回答：

```text
动态变化强度增加后，Restart、Warm-start和有限预算之间的关系是什么？
```

下一步根据这个基准结果再加入 Local Repair，并比较：

```text
Restart vs Warm-start vs Local Repair
```

## 三策略公平基准

运行：

```matlab
RunThreeStrategyBenchmark
```

这一版固定同一张三维地图、同一组障碍物、同一速度和基础时间窗，只改变动态订单集合：

- Mild：1个取消 + 1个新增；
- Moderate：2个取消 + 2个新增；
- Severe：4个取消 + 4个新增。

比较方法：

- `Restart`：全局重新初始化 Routing PSO；
- `WarmStart`：继承旧路线并插入新增订单；
- `Repair`：只做取消删除和时间窗感知的新增订单插入。

主实验采用总 Function Evaluation 上限：

```text
500 / 1000 / 1500 / 2500
```

Warm-start 的候选路线评价也计入 `warm_start_fe`，不会免费使用额外计算量。

输出：

- `results/three_strategy_benchmark_summary.csv`；
- `results/three_strategy_benchmark_result.mat`。

当前 `nRuns=1` 仅用于诊断。正式实验前还需运行可行性审计，并将 `nRuns` 提高到10或30。

## 动态状态参考可行性

新增：

```matlab
BuildReferenceSolution
VerifyDynamicStateFeasibility
```

这两个函数用于区分：

```text
算法没有在有限FE内找到可行解
```

和：

```text
事件发生后的当前位置、时间和剩余任务本身没有可行路线
```

参考结果只能称为 `reference/best-known candidate`，不能称为全局最优解。

## 目录整理（2026-09-25）

当前目录按职责整理为：

```text
src/          核心模型、路由PSO、三维航段和评价函数
experiments/  生成实例、运行三策略实验、结果分析
 audit/       MILP可行性审计和参考解工具
archive/      旧版主程序和历史绘图脚本
results/      实验输出
```

在 MATLAB 中先运行：

```matlab
cd('D:\111\Desktop\噜噜\复现\PSO_UAV_TW_3D_Dynamic_Benchmark20')
setup
```

然后再运行实验脚本。

当前主线优先使用：

```matlab
BuildAdditionDiagnosticSet
RunAcceptedPairedDiagnostic
AnalyzeDiagnosticResults
```

`archive/` 中的脚本只作为历史记录，不作为当前主流程。
