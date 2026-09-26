function main
%MAIN Benchmark20主入口：设置路径并提示当前实验主线。
setup;
fprintf('Benchmark20 paths are ready.\n');
fprintf('Dynamic baseline:\n');
fprintf('  RunThreeStrategyBenchmark\n');
fprintf('Current optimizer prototype:\n');
fprintf('  BuildTSPTWBenchmark\n');
fprintf('  ValidateTSPTWReferenceSolutions\n');
fprintf('  RunTSPTWOptimizerPrototype\n');
fprintf('  RunFixedBudgetAudit\n');
fprintf('  RunTSPTWFeasibilityAudit\n');
fprintf('Audit commands:\n');
fprintf('  RunExactDynamicAudit\n');
fprintf('  RunFeasibleConstructionAudit\n');
end
