function [bestRoute,bestCost,bestDetail,stats] = DynamicRelocateSearch( ...
    initialRoute,model,cache,searchOptions,maxFE,initialDetail)
%DYNAMICRELOCATESEARCH 动态三维航段评价下的有限FE Relocate强化
%   global-relocate：均匀随机选择源客户；
%   impact-relocate：按事件影响分数加权选择源客户，但不冻结其他客户。

if nargin<4 || isempty(searchOptions), searchOptions=struct(); end
if nargin<5 || isempty(maxFE), maxFE=inf; end
if nargin<6, initialDetail=[]; end
searchOptions=FillOptions(searchOptions,cache);

if isempty(initialDetail)
    [bestCost,bestDetail]=EvaluateSchedule(initialRoute,model,cache);
    functionEvaluations=1;
elseif isfield(initialDetail,'cost')
    bestDetail=initialDetail;
    bestCost=initialDetail.cost;
    functionEvaluations=0;
else
    [bestCost,bestDetail]=EvaluateSchedule(initialRoute,model,cache);
    functionEvaluations=1;
end
bestRoute=initialRoute(:)';
stats.functionEvaluations=functionEvaluations;
stats.improvementCount=0;
stats.priorityFE=0;
stats.globalFE=0;
stats.acceptedSourceRanks=zeros(0,1);
if functionEvaluations>=maxFE, return; end

[sourceOrder,sourceScores]=BuildSourceOrder(bestRoute,searchOptions);
candidateRoutes=cell(0,1);
candidateRanks=zeros(0,1);
candidatePriority=false(0,1);
for sourceRank=1:numel(sourceOrder)
    sourcePosition=sourceOrder(sourceRank);
    id=bestRoute(sourcePosition);
    routeWithout=bestRoute; routeWithout(sourcePosition)=[];
    insertionOrder=randperm(numel(routeWithout)+1);
    for q=insertionOrder
        candidate=[routeWithout(1:q-1),id,routeWithout(q:end)];
        if isequal(candidate,bestRoute), continue; end
        candidateRoutes{end+1,1}=candidate; %#ok<AGROW>
        candidateRanks(end+1,1)=sourceRank; %#ok<AGROW>
        candidatePriority(end+1,1)=sourceScores(sourceRank)>0; %#ok<AGROW>
    end
end

% 先按源客户排序，保证localSearchFE约等于“完整研究一个客户的插入位置”。
% 同一源客户内部的位置顺序已经随机化，减少固定位置偏差。
for k=1:numel(candidateRoutes)
    if functionEvaluations>=maxFE, break; end
    [candidateCost,candidateDetail]=EvaluateSchedule(candidateRoutes{k},model,cache);
    functionEvaluations=functionEvaluations+1;
    if candidatePriority(k), stats.priorityFE=stats.priorityFE+1; else, stats.globalFE=stats.globalFE+1; end
    if IsBetterDynamicSolution(candidateCost,candidateDetail,bestCost,bestDetail)
        bestRoute=candidateRoutes{k};
        bestCost=candidateCost;
        bestDetail=candidateDetail;
        stats.improvementCount=stats.improvementCount+1;
        stats.acceptedSourceRanks(end+1,1)=candidateRanks(k); %#ok<AGROW>
    end
end
stats.functionEvaluations=functionEvaluations;
end

function options=FillOptions(options,cache)
defaults=struct('mode','global-relocate','priorityIDs',cache.customerIDs(:)', ...
    'impactIDs',cache.customerIDs(:)','impactScores',zeros(size(cache.customerIDs(:)')), ...
    'impactAlpha',0.7);
fields=fieldnames(defaults);
for k=1:numel(fields)
    field=fields{k};
    if ~isfield(options,field) || isempty(options.(field)), options.(field)=defaults.(field); end
end
options.mode=lower(string(options.mode));
end

function [sourceOrder,sourceScores]=BuildSourceOrder(route,options)
n=numel(route);
sourceScores=zeros(1,n);
mode=lower(string(options.mode));
if mode=="impact-relocate" || mode=="feasibility-relocate"
    for k=1:n
        idx=find(options.impactIDs==route(k),1);
        if ~isempty(idx) && idx<=numel(options.impactScores)
            sourceScores(k)=max(0,options.impactScores(idx));
        end
    end
    if sum(sourceScores)<=0
        sourceScores(:)=1;
    end
    normalized=sourceScores/max(sum(sourceScores),eps);
    weights=(1-options.impactAlpha)/n+options.impactAlpha*normalized;
    keys=-log(max(rand(1,n),eps))./max(weights,eps);
    [~,sourceOrder]=sort(keys,'ascend');
else
    sourceOrder=randperm(n);
    sourceScores(:)=0;
end
sourceScores=sourceScores(sourceOrder);
end

