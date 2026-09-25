function [cache,stats] = UpdateLegCache(model,oldCache,activeCustomerIDs,startPosition)
%UPDATINGLEGCACHE 更新动态场景航段缓存
%   静态环境下，已有客户-客户和客户-home航段直接复用；
%   当前状态节点和新增客户相关航段重新计算。

timer = tic;
if nargin<4 || isempty(startPosition)
    startPosition = oldCache.startPosition;
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

reused = 0;
computed = 0;
for i = 1:cache.nNodes
    for j = 1:cache.nNodes
        if i==j
            continue;
        end
        fromType = NodeType(i,cache);
        toType = NodeType(j,cache);
        fromID = NodeID(i,cache);
        toID = NodeID(j,cache);

        canReuse = strcmp(fromType,'customer') ...
            && (strcmp(toType,'customer') || strcmp(toType,'home')) ...
            && IsCachedNode(oldCache,fromID) ...
            && IsCachedTarget(oldCache,toType,toID);

        if canReuse
            oldI = CustomerIndex(oldCache,fromID);
            if strcmp(toType,'customer')
                oldJ = CustomerIndex(oldCache,toID);
            else
                oldJ = oldCache.homeIndex;
            end
            cache.legs{i,j} = oldCache.legs{oldI,oldJ};
            reused = reused+1;
        else
            cache.legs{i,j} = Plan3DLeg( ...
                cache.nodes(i,:),cache.nodes(j,:),model);
            computed = computed+1;
        end
    end
end

stats.reusedCount = reused;
stats.computedCount = computed;
stats.onlineLegTime = toc(timer);
end

function type = NodeType(index,cache)
if index==cache.startIndex
    type = 'start';
elseif index==cache.homeIndex
    type = 'home';
else
    type = 'customer';
end
end

function id = NodeID(index,cache)
if strcmp(NodeType(index,cache),'customer')
    id = cache.customerIDs(index-1);
else
    id = 0;
end
end

function tf = IsCachedNode(cache,id)
tf = any(cache.customerIDs==id);
end

function tf = IsCachedTarget(cache,type,id)
if strcmp(type,'home')
    tf = true;
else
    tf = any(cache.customerIDs==id);
end
end

function index = CustomerIndex(cache,id)
index = find(cache.customerIDs==id,1)+1;
end
