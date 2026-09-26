function [bestRoute,bestDetail,stats] = DiscreteRouteSearch(initialRoute,instance,options,initialDetail)
%DISCRETEROUTESEARCH 公平的TSPTW离散邻域搜索
%   relocate/global：随机选择source客户，并完整评价其所有插入位置；
%   late：按当前客户迟到量选择source；
%   propagation：按迟到后缀传播分数选择source；
%   swap、2opt、mixed用于算子消融。

if nargin<3 || isempty(options), options=struct(); end
if nargin<4, initialDetail=[]; end
if ~isfield(options,'mode') || isempty(options.mode), options.mode='relocate'; end
if ~isfield(options,'maxFE') || isempty(options.maxFE), options.maxFE=inf; end
if ~isfield(options,'latePenalty') || isempty(options.latePenalty), options.latePenalty=1000; end
if ~isfield(options,'waitPenalty') || isempty(options.waitPenalty), options.waitPenalty=0; end

bestRoute=initialRoute(:)';
if isempty(initialDetail)
    [bestCost,bestDetail]=EvaluateTSPTWRoute(bestRoute,instance,options);
    functionEvaluations=1;
else
    bestDetail=initialDetail; bestCost=bestDetail.cost; functionEvaluations=0;
end
stats.functionEvaluations=functionEvaluations; stats.firstFeasibleEvaluation=inf;
stats.firstFeasibleRoute=[]; stats.firstFeasibleDetail=[]; stats.improvementCount=0;
stats.relocateFE=0; stats.swapFE=0; stats.twoOptFE=0;
stats.relocateAccept=0; stats.swapAccept=0; stats.twoOptAccept=0;
stats.moves=strings(0,1);
mode=lower(string(options.mode));
if mode=="full", mode="mixed"; end
switch mode
    case {"relocate","global","late","propagation","feasibility"}
        operators="relocate";
        if mode=="relocate", mode="global"; end
        if mode=="feasibility", mode="propagation"; end
    case "swap", operators="swap";
    case "2opt", operators="2opt";
    case "mixed", operators=["relocate","swap","2opt"];
    otherwise, error('不支持的离散搜索模式：%s。',options.mode);
end

while functionEvaluations<options.maxFE
    [candidatePools,movePools]=BuildCandidatePools(bestRoute,bestDetail,operators,mode);
    nextIndex=ones(1,numel(operators)); operatorOrder=randperm(numel(operators));
    passBestRoute=bestRoute; passBestDetail=bestDetail; passBestCost=bestCost;
    passMove=""; passOperator=""; passFE=0;
    while functionEvaluations<options.maxFE && any(nextIndex<=cellfun(@numel,candidatePools))
        for orderIndex=1:numel(operatorOrder)
            opIndex=operatorOrder(orderIndex);
            if nextIndex(opIndex)>numel(candidatePools{opIndex}), continue; end
            candidate=candidatePools{opIndex}{nextIndex(opIndex)};
            moveName=movePools{opIndex}(nextIndex(opIndex));
            nextIndex(opIndex)=nextIndex(opIndex)+1;
            [candidateCost,candidateDetail]=EvaluateTSPTWRoute(candidate,instance,options);
            functionEvaluations=functionEvaluations+1; passFE=passFE+1;
            if isinf(stats.firstFeasibleEvaluation) && candidateDetail.isFeasible
                stats.firstFeasibleEvaluation=functionEvaluations;
                stats.firstFeasibleRoute=candidate; stats.firstFeasibleDetail=candidateDetail;
            end
            stats=AddOperatorFE(stats,operators(opIndex));
            if IsBetterSolution(candidateCost,candidateDetail,passBestCost,passBestDetail)
                passBestRoute=candidate; passBestDetail=candidateDetail;
                passBestCost=candidateCost; passMove=moveName;
                passOperator=operators(opIndex);
            end
            if functionEvaluations>=options.maxFE, break; end
        end
    end
    if IsBetterSolution(passBestCost,passBestDetail,bestCost,bestDetail)
        bestRoute=passBestRoute; bestDetail=passBestDetail; bestCost=passBestCost;
        stats.improvementCount=stats.improvementCount+1;
        stats.moves(end+1,1)=passMove; %#ok<AGROW>
        stats=AddOperatorAccept(stats,passOperator);
    else
        break;
    end
    if passFE==0, break; end
end
stats.functionEvaluations=functionEvaluations;
end

function [candidatePools,movePools]=BuildCandidatePools(route,detail,operators,mode)
candidatePools=cell(1,numel(operators)); movePools=cell(1,numel(operators));
for k=1:numel(operators)
    if operators(k)=="relocate" && any(mode==["global","late","propagation"])
        [candidates,moves]=BuildSourceRelocate(route,detail,mode);
        order=1:numel(candidates);
    else
        [candidates,moves]=BuildNeighborhood(route,operators(k));
        order=randperm(numel(candidates));
    end
    candidatePools{k}=candidates(order); movePools{k}=moves(order);
end
end

function [candidates,moves]=BuildSourceRelocate(route,detail,mode)
route=route(:)'; n=numel(route); scores=zeros(1,n);
if mode=="late" || mode=="propagation"
    if isfield(detail,'records') && ~isempty(detail.records)
        suffix=zeros(size(detail.records,1),1);
        for k=size(detail.records,1):-1:1
            suffix(k)=detail.records(k,4);
            if k<size(detail.records,1), suffix(k)=suffix(k)+suffix(k+1); end
            idx=find(route==detail.records(k,1),1);
            if ~isempty(idx)
                if mode=="late"
                    scores(idx)=max(0,detail.records(k,4));
                else
                    scores(idx)=max(0,detail.records(k,4))+ ...
                        0.5*suffix(k)/max(n-k+1,1);
                end
            end
        end
    end
    if max(scores)<=0, scores(:)=1; end
    [~,sourceOrder]=sort(scores,'descend');
else
    sourceOrder=randperm(n);
end
candidates=cell(0,1); moves=strings(0,1);
for i=sourceOrder
    remaining=route([1:i-1,i+1:n]);
    positionOrder=randperm(numel(remaining)+1);
    for j=positionOrder
        candidate=[remaining(1:j-1),route(i),remaining(j:end)];
        if isequal(candidate,route), continue; end
        candidates{end+1,1}=candidate; %#ok<AGROW>
        moves(end+1,1)="relocate("+i+","+j+")"; %#ok<AGROW>
    end
end
end

function [candidates,moves]=BuildNeighborhood(route,neighborhood)
route=route(:)'; n=numel(route); candidates=cell(0,1); moves=strings(0,1);
switch lower(char(neighborhood))
    case 'relocate'
        for i=1:n
            remaining=route([1:i-1,i+1:n]);
            for j=0:numel(remaining)
                candidate=[remaining(1:j),route(i),remaining(j+1:end)];
                if isequal(candidate,route), continue; end
                candidates{end+1,1}=candidate; %#ok<AGROW>
                moves(end+1,1)="relocate("+i+","+j+")"; %#ok<AGROW>
            end
        end
    case 'swap'
        for i=1:n-1
            for j=i+1:n
                candidate=route; candidate([i,j])=candidate([j,i]);
                candidates{end+1,1}=candidate; %#ok<AGROW>
                moves(end+1,1)="swap("+i+","+j+")"; %#ok<AGROW>
            end
        end
    case '2opt'
        for i=1:n-1
            for j=i+1:n
                candidate=route; candidate(i:j)=route(j:-1:i);
                candidates{end+1,1}=candidate; %#ok<AGROW>
                moves(end+1,1)="2opt("+i+","+j+")"; %#ok<AGROW>
            end
        end
end
end

function tf=IsBetterSolution(candidateCost,candidateDetail,bestCost,bestDetail)
if candidateDetail.totalLate<bestDetail.totalLate-1e-10
    tf=true;
elseif abs(candidateDetail.totalLate-bestDetail.totalLate)<=1e-10
    tf=candidateDetail.tourCost<bestDetail.tourCost-1e-10;
else
    tf=false;
end
end

function stats=AddOperatorFE(stats,operator)
switch operator
    case "relocate", stats.relocateFE=stats.relocateFE+1;
    case "swap", stats.swapFE=stats.swapFE+1;
    case "2opt", stats.twoOptFE=stats.twoOptFE+1;
end
end

function stats=AddOperatorAccept(stats,operator)
switch operator
    case "relocate", stats.relocateAccept=stats.relocateAccept+1;
    case "swap", stats.swapAccept=stats.swapAccept+1;
    case "2opt", stats.twoOptAccept=stats.twoOptAccept+1;
end
end
