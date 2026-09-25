function [affectedIDs,info] = DetectAffectedRegion(previousRoute,activeIDs, ...
    eventCustomerIDs,model,cache)
%DETECTAFFECTEDREGIONV2 用直接插入后的时间传播识别受影响区域
%   旧客户会因新增订单插入而受到影响，不再只返回新增客户。
activeIDs=activeIDs(:)';
previousRoute=previousRoute(:)';
eventCustomerIDs=eventCustomerIDs(:)';
oldRoute=previousRoute(ismember(previousRoute,activeIDs));
if isempty(oldRoute), oldRoute=activeIDs; end
[oldCost,oldDetail]=EvaluateSchedule(oldRoute,model,cache);

% 先执行便宜的直接插入，得到事件传播后的基线路线。
repairRoute=oldRoute;
for id=eventCustomerIDs
    bestCost=inf; bestRoute=repairRoute;
    for pos=1:numel(repairRoute)+1
        candidate=[repairRoute(1:pos-1),id,repairRoute(pos:end)];
        [cost,~]=EvaluateSchedule(candidate,model,cache);
        if cost<bestCost
            bestCost=cost; bestRoute=candidate;
        end
    end
    repairRoute=bestRoute;
end
[repairCost,repairDetail]=EvaluateSchedule(repairRoute,model,cache);

affected=intersect(eventCustomerIDs,activeIDs,'stable');
slackBefore=SlackMap(oldDetail,model,oldRoute);
slackAfter=SlackMap(repairDetail,model,repairRoute);
propagation=struct('id',{},'slackBefore',{},'slackAfter',{},'pressure',{});
oldIDs=intersect(oldRoute,repairRoute,'stable');
for id=oldIDs
    before=GetSlack(slackBefore,id);
    after=GetSlack(slackAfter,id);
    pressure=max(0,before-after)/max(before,eps);
    propagation(end+1)=struct('id',id,'slackBefore',before, ...
        'slackAfter',after,'pressure',pressure); %#ok<AGROW>
    if pressure>=0.25 || after<60
        affected=union(affected,id,'stable');
    end
end

% 加入新增客户在直接修复路线中的前驱和后继。
for id=eventCustomerIDs
    pos=find(repairRoute==id,1);
    if isempty(pos), continue; end
    neighbors=repairRoute(max(1,pos-1):min(numel(repairRoute),pos+1));
    affected=union(affected,neighbors,'stable');
end

if isempty(affected)
    affected=activeIDs(1:min(3,numel(activeIDs)));
end
affectedIDs=intersect(affected,activeIDs,'stable');
info.previousRoute=oldRoute;
info.repairRoute=repairRoute;
info.oldCost=oldCost;
info.repairCost=repairCost;
info.slackBefore=slackBefore;
info.slackAfter=slackAfter;
info.propagation=propagation;
info.affectedCount=numel(affectedIDs);
info.activeCount=numel(activeIDs);
info.affectedRatio=numel(affectedIDs)/max(numel(activeIDs),1);
end

function map=SlackMap(detail,model,route)
map=zeros(max([route(:);0])+1,1);
for k=1:size(detail.records,1)
    id=detail.records(k,1);
    map(id)=model.windows(id,2)-detail.records(k,3);
end
end
function value=GetSlack(map,id)
if id<=numel(map) && map(id)~=0, value=map(id); else, value=0; end
end
