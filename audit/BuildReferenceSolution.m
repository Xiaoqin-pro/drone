function report = BuildReferenceSolution(model,cache,activeIDs,refBudget,nRestarts)
%BUILDREFERENCESOLUTION 用较大预算生成参考可行解候选
%   结果只能称为reference/best-known candidate，不称为全局最优。
if nargin<4 || isempty(refBudget)
    refBudget = 6000;
end
if nargin<5 || isempty(nRestarts)
    nRestarts = 2;
end

nPop = 60;
maxIt = max(0,floor(refBudget/nPop)-1);
bestCost = inf;
bestSolution = [];
bestStats = [];
feasibleFound = false;
totalFE = 0;

model.cfg.silent = true;
for r = 1:nRestarts
    rng(900000+r,'twister');
    [solution,~,stats] = RoutingPSO(model,cache,maxIt,nPop, ...
        0.90,0.995,1.7,1.7,[]);
    totalFE = totalFE+stats.functionEvaluations;
    if solution.Cost<bestCost
        bestCost = solution.Cost;
        bestSolution = solution;
        bestStats = stats;
    end
    if solution.Detail.isFeasible
        feasibleFound = true;
        if isempty(bestSolution) || ~bestSolution.Detail.isFeasible ...
                || solution.Cost<bestSolution.Cost
            bestSolution = solution;
            bestCost = solution.Cost;
            bestStats = stats;
        end
    end
end

report.isFound = feasibleFound;
report.referenceCost = bestCost;
report.referenceSolution = bestSolution;
report.referenceStats = bestStats;
report.totalFE = totalFE;
report.refBudget = refBudget;
report.nRestarts = nRestarts;
report.activeIDs = activeIDs(:)';
end
