function [bestRoute,bestDetail,stats] = CompetitiveNeighborhoodSearch(route,instance,maxFE,initialDetail,options)
%COMPETITIVENEIGHBORHOODSEARCH 同一incumbent上的公平异构邻域竞争
%   每个邻域最多使用quota=n-1 FE，分别从同一个base route开始；
%   最后只接受三种邻域结果中的最好者，不顺序传播中间路线。

if nargin<5 || isempty(options), options=struct(); end
if nargin<4 || isempty(initialDetail)
    [baseCost,baseDetail]=EvaluateTSPTWRoute(route,instance,options);
else
    baseDetail=initialDetail; baseCost=initialDetail.cost;
end
if nargin<3 || isempty(maxFE), maxFE=3*max(1,instance.nCustomers-1); end
quota=max(1,instance.nCustomers-1);
baseRoute=route(:)';
orders=["2opt","swap","relocate"];
bestRoute=baseRoute; bestDetail=baseDetail; bestCost=baseCost;
functionEvaluations=0; stats.twoOptFE=0; stats.swapFE=0; stats.relocateFE=0;
stats.twoOptAccept=0; stats.swapAccept=0; stats.relocateAccept=0;
stats.neighborhoodBestCost=zeros(1,3); stats.neighborhoodImprovement=zeros(1,3);
for k=1:numel(orders)
    if functionEvaluations>=maxFE, break; end
    [candidates,moves]=BuildCandidates(baseRoute,orders(k)); %#ok<ASGLU>
    order=randperm(numel(candidates)); localRoute=baseRoute; localDetail=baseDetail; localCost=baseCost;
    localImprovement=false; localBestGain=0;
    for q=order(1:min(numel(order),quota))
        if functionEvaluations>=maxFE, break; end
        [candidateCost,candidateDetail]=EvaluateTSPTWRoute(candidates{q},instance,options);
        functionEvaluations=functionEvaluations+1;
        stats=AddFE(stats,orders(k));
        if IsBetter(candidateCost,candidateDetail,localCost,localDetail)
            localRoute=candidates{q}; localDetail=candidateDetail; localCost=candidateCost;
            localImprovement=true; localBestGain=max(localBestGain,baseCost-localCost);
        end
    end
    stats.neighborhoodBestCost(k)=localCost;
    stats.neighborhoodImprovement(k)=max(0,baseCost-localCost);
    if localImprovement, stats=AddAccept(stats,orders(k)); end
    if IsBetter(localCost,localDetail,bestCost,bestDetail)
        bestRoute=localRoute; bestDetail=localDetail; bestCost=localCost;
    end
end
stats.functionEvaluations=functionEvaluations;
stats.competitiveWinner=orders(FindWinner(stats.neighborhoodBestCost,baseCost,bestCost));
end

function index=FindWinner(costs,baseCost,bestCost)
[~,index]=min(costs); %#ok<ASGLU>
for k=1:numel(costs)
    if abs(costs(k)-bestCost)<=1e-10, index=k; return; end
end
end

function [candidates,moves]=BuildCandidates(route,neighborhood)
route=route(:)'; n=numel(route); candidates=cell(0,1); moves=strings(0,1);
switch lower(char(neighborhood))
    case 'relocate'
        for i=1:n
            remaining=route([1:i-1,i+1:n]);
            for j=0:numel(remaining)
                candidate=[remaining(1:j),route(i),remaining(j+1:end)];
                if isequal(candidate,route), continue; end
                candidates{end+1,1}=candidate; moves(end+1,1)="relocate"; %#ok<AGROW>
            end
        end
    case 'swap'
        for i=1:n-1
            for j=i+1:n
                candidate=route; candidate([i,j])=candidate([j,i]);
                candidates{end+1,1}=candidate; moves(end+1,1)="swap"; %#ok<AGROW>
            end
        end
    case '2opt'
        for i=1:n-1
            for j=i+1:n
                candidate=route; candidate(i:j)=route(j:-1:i);
                candidates{end+1,1}=candidate; moves(end+1,1)="2opt"; %#ok<AGROW>
            end
        end
end
end

function tf=IsBetter(candidateCost,candidateDetail,bestCost,bestDetail)
if candidateDetail.totalLate<bestDetail.totalLate-1e-10
    tf=true;
elseif abs(candidateDetail.totalLate-bestDetail.totalLate)<=1e-10
    tf=candidateDetail.tourCost<bestDetail.tourCost-1e-10;
else
    tf=false;
end
end

function stats=AddFE(stats,neighborhood)
switch lower(char(neighborhood))
    case '2opt', stats.twoOptFE=stats.twoOptFE+1;
    case 'swap', stats.swapFE=stats.swapFE+1;
    case 'relocate', stats.relocateFE=stats.relocateFE+1;
end
end
function stats=AddAccept(stats,neighborhood)
switch lower(char(neighborhood))
    case '2opt', stats.twoOptAccept=stats.twoOptAccept+1;
    case 'swap', stats.swapAccept=stats.swapAccept+1;
    case 'relocate', stats.relocateAccept=stats.relocateAccept+1;
end
end
