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

## 优化器机制校准实验

当前已经加入两类机制门槛实验：

```matlab
RunFixedBudgetAudit
RunTSPTWFeasibilityAudit
```

`RunFixedBudgetAudit` 比较统一固定局部搜索深度 `5/15/25/35/40` 与 Adaptive，检查自适应预算是否真的优于“固定多做一些Relocate”。

`RunTSPTWFeasibilityAudit` 在没有Warm-start可行旧计划的标准TSPTW实例上，比较：

```text
Random-key PSO
PSO + GlobalRelocate
PSO + FeasibilityRelocate
```

重点指标是：

```text
FeasibleRate
FirstFeasibleFE
TotalLate
```

当前这两组实验用于决定哪些机制能够进入最终Proposed，而不是直接制造更多算法组件。

## 规模一致的 Source-wise Relocate

TSPTW局部搜索现在把一次Relocate动作定义为：

```text
选择一个source customer
→ 完整评价该客户的所有reinsertion positions
```

Global、LateOnly和Propagation三种模式只改变source customer的选择顺序，不改变单个source的邻域大小。这样不同客户规模下的局部搜索强度更一致。

Propagation模式使用：

```text
当前客户迟到量 + 后缀迟到传播分数
```

用于优先选择时间窗传播影响较大的客户。

## Core Mechanism Freeze 实验

新增：

```matlab
RunCoreMechanismFreezeTSPTW
```

在标准TSPTW上比较：

```text
PSO
PSO + GlobalRelocate
PSO + LateOnlyRelocate
PSO + PropagationRelocate
PSO + StateSwitch
```

局部搜索以一个source customer的完整reinsertion neighborhood为基本单元，预算使用 `25n / 50n / 100n`。结果重点记录可行率、首次可行FE、总迟到和BKS Gap。

这组实验用于决定Propagation和StateSwitch能否进入最终Proposed，不与动态三维应用实验混为一谈。

## 因果机制审计

为区分“Relocate有效”和“source选择有效”，新增：

```matlab
RunSourceSelectionAudit
RunStateSwitchForkAudit
```

`RunSourceSelectionAudit` 在相同的不可行路线集上分别测试Global、LateOnly和Propagation source，不经过PSO轨迹，直接比较一次完整source neighborhood的迟到下降和可行性恢复。

`RunStateSwitchForkAudit` 从同一条first-feasible路线分叉：

```text
AlwaysPropagation
StateSwitchToGlobal
```

用于单独测量进入可行域后切换到Global Relocate是否继续改善路线质量。

## Discrete-to-Swarm Feedback 原型

为检验离散Relocate经验能否反向增强PSO，新增：

```matlab
RunFeedbackAblationTSPTW
```

比较：

```text
PSO
PSO + Relocate
PSO + StateSwitch
PSO + Relocate + Feedback
StateSwitch + Feedback
```

当前结果用于机制筛选。若Feedback不能在相同FE下稳定优于不带Feedback的对应方法，则不将其保留为Proposed核心，仅作为负结果和补充分析。

## Feasible Source Oracle Audit

新增机制诊断实验：

```matlab
RunFeasibleSourceOracleAudit
```

该实验不改变Proposed算法，而是对同一条可行路线离线完整评价每个source customer的reinsertion neighborhood，得到oracle-best source，再比较Random和RemovalSaving等简单source规则的：

```text
Oracle rank
Normalized regret
可行路线质量改善
```

该实验用于判断可行状态下是否存在稳定的quality-oriented source information，避免直接凭直觉设计第二个状态机制。

## Feasible Source Oracle Audit V2

当前Oracle审计已扩展为：

```matlab
RunFeasibleSourceOracleAudit
```

V2收集每个种子的：

```text
first-feasible route
final-feasible route
reference route
```

并对每条路线完整评价所有source neighborhood，记录：

```text
oracle gain
random expected gain
random improve probability
actionable state
source gain
removal saving
slack
waiting
```

这些结果用于先诊断可行状态下是否存在可利用的quality-oriented source information，再决定是否设计新的source规则。

## Feasible Source Feature Audit V1

新增：

```matlab
RunFeasibleSourceFeatureAudit
```

该实验以路线为统计单位，而不是把所有source行错误地当作独立样本。对每条actionable feasible route分别计算：

```text
RemovalSaving
Slack
Waiting
LocalTravel
```

并报告route-wise Spearman、Oracle hit、Top-3、normalized rank和gain capture。

Feature Audit只用于决定可行状态下是否存在稳定quality-oriented source信息，不直接引入新的算法公式。

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

