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
- 统一 Function Evaluation（FE）预算。

## 运行

```matlab
cd('D:\111\Desktop\噜噜\复现\PSO_UAV_TW_3D_Dynamic_Benchmark20')
setup
```

动态基线：

```matlab
RunThreeStrategyBenchmark
```

当前主线的原生 TSPTW 优化器原型：

```matlab
BuildTSPTWBenchmark
ValidateTSPTWReferenceSolutions
RunTSPTWOptimizerPrototype
```

## 原生 TSPTW 优化主线

当前已接入 Solomon-Potvin-Bengio 格式的 TSPTW 实例，作为“优化器能力”开发集。它与三维无人机模型分开，先验证组合路径搜索本身：

```text
ReadTSPTWInstance       读取距离矩阵和时间窗
EvaluateTSPTWRoute      计算旅行代价、等待、迟到和惩罚适应度
ReadTSPTWReferenceSolutions 读取官方参考路线
ValidateTSPTWReferenceSolutions 校准读取器和评价器
RandomKeyTSPTWPSO       Random-key PSO 基线
DiscreteRouteSearch     Relocate / Swap / 2-opt 离散邻域
```

原型实验在相同实例、相同随机种子和相同 FE 预算下比较：

```text
Random-key PSO
PSO + Relocate
PSO + Swap
PSO + 2-opt
PSO + Mixed
```

开发数据位于：

```text
data/tsp_tw_raw/                  原始TSPTW实例
data/tsp_tw_benchmark/            当前固定的20/32/46节点开发清单
```

结果位于：

```text
results/tsptw_optimizer_prototype_summary.csv
results/tsptw_optimizer_prototype_result.mat
results/tsptw_optimizer_prototype_anytime.png
```

这一步的论文意义是先回答：

```text
问题特定的离散邻域，能否在相同FE预算下改善Random-key PSO的组合搜索质量？
```

当前原型已经加入可行性优先比较，并对 Relocate、Swap、2-opt 和 Mixed 做公平FE消融。官方参考路线校验结果保存在 `results/tsptw_reference_validation.csv`。

静态 TSPTW 原型验证通过后，再把同一套离散搜索机制接回动态订单和三维 UAV 验证场景。

## 动态三维主线原型

当前已把 Relocate 接回原来的三维动态无人机链条，新增：

```text
src/DynamicRelocateSearch.m
experiments/RunDynamicMemeticPrototype.m
```

动态原型只比较三种方法：

```text
Repair
WarmPSO
WarmPSO + Relocate
```

Relocate 的每次候选路线评价都重新调用 `EvaluateSchedule`，因此会重新计算三维安全航段、飞行距离、到达时间、时间窗迟到和障碍物约束。当前原型使用8个代表性动态实例、10个配对种子和500/1000/1500/2500严格FE检查点。

运行：

```matlab
RunDynamicMemeticPrototype
```

结果：

```text
results/dynamic_memetic_prototype_summary.csv
results/dynamic_memetic_prototype_result.mat
results/dynamic_memetic_prototype_curves.png
```

这一版仍是主线回归验证，不是最终自适应算法。下一步再将 `ComputeImpactProfile` 接入 Relocate 的客户采样优先级，比较全局Relocate和Impact-guided Relocate。

## Impact-guided Relocate 原型

在动态三维主线中，新增了全局Relocate与事件影响引导Relocate的对照实验：

```matlab
RunImpactGuidedRelocatePrototype
```

比较方法：

```text
Repair
WarmPSO
WarmPSO + GlobalRelocate
WarmPSO + ImpactRelocate
```

影响引导不会冻结低影响客户，而是根据 `ComputeImpactProfile` 对Relocate源客户进行加权抽样；影响画像的评价FE会计入总预算。当前结果仍属于机制原型，不直接作为最终论文结果。

输出：

```text
results/impact_guided_relocate_summary.csv
results/impact_guided_relocate_result.mat
results/impact_guided_relocate_curves.png
```

`BuildPreEventSolution.m`用于重建动态基准生成时的事件前旧计划，保证事件状态、旧路线和重规划算法使用同一历史计划。

## 面向优化算法论文的 Adaptive Memetic PSO 原型

当前主线进一步抽象为两个算法机制：

```text
Feasibility-State Adaptive Search
Adaptive FE Allocation
```

新增：

```text
src/AdaptiveMemeticRoutingPSO.m
experiments/RunAdaptiveMemeticPrototype.m
```

算法根据当前全局最优解状态自动切换：

```text
不可行：Feasibility-Restoration Relocate
可行：Quality-Intensification Relocate
```

同时根据最近单位FE的PSO收益和Relocate收益，动态决定下一轮局部搜索预算，不再固定每轮25个Relocate FE。

原型实验比较：

```text
Repair
WarmPSO
WarmPSO + FixedRelocate
AdaptiveMemeticPSO
```

运行：

```matlab
RunAdaptiveMemeticPrototype
```

结果：

```text
results/adaptive_memetic_prototype_summary.csv
results/adaptive_memetic_prototype_result.mat
results/adaptive_memetic_prototype_curves.png
```

这一版是优化器机制原型。当前动态UAV结果用于验证机制，后续还需要在标准TSPTW/VRPTW基准上进行独立算法验证、增加强组合优化基线和统计检验。

## 结果说明

动态基线结果保存在 `results`，包括：

- `dynamic_benchmark20_summary.csv`：每个条件和每个动态事件的详细结果；
- `dynamic_benchmark20_means.csv`：分组汇总；
- `dynamic_benchmark20_result.mat`：完整实验数据。

TSPTW 原型结果只代表当前开发集和当前参数，不作为论文最终结论。正式实验仍需增加实例数量、独立随机种子、统计检验和统一运行记录。

## 当前研究定位

当前主线已经从“Repair/Warm/Restart谁更好”转向：

```text
优化算法是主角，三维动态无人机配送是验证场景。
```

近期优先级：

```text
静态TSPTW优化能力
→ 离散邻域消融
→ 动态TSPTW重规划
→ 三维UAV映射
```

暂不扩展多无人机、MOPSO、ALNS和复杂自适应选择器。

