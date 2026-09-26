function tf = IsBetterDynamicSolution(candidateCost,candidateDetail,bestCost,bestDetail)
%ISBETTERDYNAMICSOLUTION 动态三维路线的统一字典序比较器
%   可行解优先；都不可行时依次比较时间窗、障碍物、地形违反量和惩罚适应度。

if isempty(bestDetail)
    tf = true;
    return;
end
candidateFeasible = isfield(candidateDetail,'isFeasible') && candidateDetail.isFeasible;
bestFeasible = isfield(bestDetail,'isFeasible') && bestDetail.isFeasible;
if candidateFeasible~=bestFeasible
    tf = candidateFeasible;
    return;
end
if candidateFeasible
    tf = candidateCost<bestCost-1e-10;
    return;
end
candidateVector = [SafeField(candidateDetail,'totalLate'), ...
    SafeField(candidateDetail,'totalObstacleViolation'), ...
    SafeField(candidateDetail,'totalTerrainViolation'),candidateCost];
bestVector = [SafeField(bestDetail,'totalLate'), ...
    SafeField(bestDetail,'totalObstacleViolation'), ...
    SafeField(bestDetail,'totalTerrainViolation'),bestCost];
tf = LexicographicallySmaller(candidateVector,bestVector);
end

function value = SafeField(detail,name)
if isfield(detail,name) && isfinite(detail.(name))
    value = detail.(name);
else
    value = inf;
end
end

function tf = LexicographicallySmaller(a,b)
tf = false;
for k = 1:numel(a)
    if a(k)<b(k)-1e-10
        tf = true;
        return;
    elseif a(k)>b(k)+1e-10
        return;
    end
end
end
