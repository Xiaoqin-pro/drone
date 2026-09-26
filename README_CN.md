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
RunTSPTWOptimizerPrototype
```

## 原生 TSPTW 优化主线

当前已接入 Solomon-Potvin-Bengio 格式的 TSPTW 实例，作为“优化器能力”开发集。它与三维无人机模型分开，先验证组合路径搜索本身：

```text
ReadTSPTWInstance       读取距离矩阵和时间窗
EvaluateTSPTWRoute      计算距离、等待、迟到和惩罚适应度
RandomKeyTSPTWPSO       Random-key PSO 基线
DiscreteRouteSearch     Relocate / Swap / 2-opt 离散邻域
```

原型实验在相同实例、相同随机种子和相同 FE 预算下比较：

```text
Random-key PSO
PSO + Relocate
PSO + Relocate + Swap + 2-opt
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

静态 TSPTW 原型验证通过后，再把同一套离散搜索机制接回动态订单和三维 UAV 验证场景。

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

