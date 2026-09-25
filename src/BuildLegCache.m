function cache = BuildLegCache(model,activeCustomerIDs,startPosition)
%BUILDLEGCACHE 构建当前状态到活动客户和返航点的航段缓存
%   节点结构：1=start current state，2...n+1=customers，n+2=home。

if nargin<2 || isempty(activeCustomerIDs)
    activeCustomerIDs = model.customerIDs;
end
if nargin<3 || isempty(startPosition)
    startPosition = model.depot;
end
activeCustomerIDs = activeCustomerIDs(:)';

cache.customerIDs = activeCustomerIDs;
cache.startPosition = startPosition;
cache.homePosition = model.homePosition;
cache.startIndex = 1;
cache.customerIndex = 2:(numel(activeCustomerIDs)+1);
cache.homeIndex = numel(activeCustomerIDs)+2;
cache.nodes = [startPosition;model.customerXYZ(activeCustomerIDs,:); ...
    model.homePosition];
cache.nNodes = size(cache.nodes,1);
cache.legs = cell(cache.nNodes,cache.nNodes);
cache.reusedCount = 0;
cache.newCount = 0;

for i = 1:cache.nNodes
    for j = 1:cache.nNodes
        if i~=j
            cache.legs{i,j} = Plan3DLeg( ...
                cache.nodes(i,:),cache.nodes(j,:),model);
            cache.newCount = cache.newCount+1;
        end
    end
end
end
