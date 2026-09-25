function disruption = ComputeRouteDisruption(oldRoute,newRoute)
%COMPUTEROUTEDISRUPTION 只比较事件前后共同保留客户的顺序变化
oldRoute = oldRoute(:)';
newRoute = newRoute(:)';
commonIDs = intersect(oldRoute,newRoute,'stable');
if isempty(commonIDs)
    disruption = 0;
    return;
end
oldCommon = oldRoute(ismember(oldRoute,commonIDs));
newCommon = newRoute(ismember(newRoute,commonIDs));
commonLength = LongestCommonSubsequenceLength(oldCommon,newCommon);
disruption = 1-commonLength/numel(commonIDs);
end

function lengthValue = LongestCommonSubsequenceLength(a,b)
dp = zeros(numel(a)+1,numel(b)+1);
for i = 1:numel(a)
    for j = 1:numel(b)
        if a(i)==b(j)
            dp(i+1,j+1) = dp(i,j)+1;
        else
            dp(i+1,j+1) = max(dp(i,j+1),dp(i+1,j));
        end
    end
end
lengthValue = dp(end,end);
end
