function profile = ComputeImpactProfile(previousRoute,activeIDs,eventIDs,model,cache)
%COMPUTEIMPACTPROFILE 计算事件后旧路线的传播影响排序
activeIDs=activeIDs(:)'; previousRoute=previousRoute(:)'; eventIDs=eventIDs(:)';
oldRoute=previousRoute(ismember(previousRoute,activeIDs));
[oldCost,oldDetail]=EvaluateSchedule(oldRoute,model,cache);

% 直接插入：每个新增任务选择当前最优插入位置。
repairRoute=oldRoute; preprocessFE=1;
for id=eventIDs
    bestCost=inf; bestRoute=repairRoute;
    for pos=1:numel(repairRoute)+1
        candidate=[repairRoute(1:pos-1),id,repairRoute(pos:end)];
        [cost,~]=EvaluateSchedule(candidate,model,cache); preprocessFE=preprocessFE+1;
        if cost<bestCost, bestCost=cost; bestRoute=candidate; end
    end
    repairRoute=bestRoute;
end
[repairCost,repairDetail]=EvaluateSchedule(repairRoute,model,cache); preprocessFE=preprocessFE+1;

oldSlack=SlackMap(oldDetail,model); repairSlack=SlackMap(repairDetail,model);
oldIDs=intersect(oldRoute,repairRoute,'stable');
records=struct('id',{},'pressure',{},'slackLoss',{},'neighbor',{});
for id=oldIDs
    before=GetSlack(oldSlack,id); after=GetSlack(repairSlack,id);
    pressure=max(0,before-after)/max(before,eps);
    neighbor=0;
    for newID=eventIDs
        p=find(repairRoute==newID,1);
        if ~isempty(p)
            neighbors=repairRoute(max(1,p-1):min(numel(repairRoute),p+1));
            neighbor=neighbor || ismember(id,neighbors);
        end
    end
    records(end+1)=struct('id',id,'pressure',pressure, ...
        'slackLoss',max(0,before-after),'neighbor',neighbor); %#ok<AGROW>
end
if isempty(records)
    rankedOld=[];
else
    score=[records.pressure]+0.25*[records.neighbor];
    [~,order]=sort(score,'descend'); rankedOld=[records(order).id];
end
profile.oldRoute=oldRoute;
profile.repairRoute=repairRoute;
profile.oldCost=oldCost;
profile.repairCost=repairCost;
profile.records=records;
profile.rankedOldIDs=rankedOld;
profile.eventIDs=intersect(eventIDs,activeIDs,'stable');
profile.preprocessFE=preprocessFE;
profile.affectedIDs=unique([profile.eventIDs,rankedOld],'stable');
end

function map=SlackMap(detail,model)
map=nan(max(detail.routeIDs)+1,1);
for k=1:size(detail.records,1)
    id=detail.records(k,1); map(id)=model.windows(id,2)-detail.records(k,3);
end
end
function x=GetSlack(map,id)
if id<=numel(map) && isfinite(map(id)), x=map(id); else, x=0; end
end
