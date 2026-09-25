function [affectedIDs,info] = DetectAffectedRegion(previousRoute,activeIDs, ...
    eventCustomerIDs,model,cache)
%DETECTAFFECTEDREGION 根据动态订单和时间窗压力确定初始受影响区域
activeIDs = activeIDs(:)';
previousRoute = previousRoute(:)';
eventCustomerIDs = eventCustomerIDs(:)';
affected = intersect(eventCustomerIDs,activeIDs,'stable');
oldRoute = previousRoute(ismember(previousRoute,activeIDs));
if isempty(oldRoute), oldRoute=activeIDs; end
[~,oldDetail] = EvaluateSchedule(oldRoute,model,cache);
criticalIDs=[];
for k=1:size(oldDetail.records,1)
    customerID=oldDetail.records(k,1);
    slack=model.windows(customerID,2)-oldDetail.records(k,3);
    if slack<60, criticalIDs(end+1)=customerID; end %#ok<AGROW>
end
affected=union(affected,criticalIDs,'stable');
for id=eventCustomerIDs
    pos=find(previousRoute==id,1);
    if isempty(pos), continue; end
    affected=union(affected,previousRoute(max(1,pos-2):min(numel(previousRoute),pos+2)),'stable');
end
if isempty(affected)
    if isempty(criticalIDs), affected=activeIDs(1:min(3,numel(activeIDs)));
    else, affected=criticalIDs(1:min(3,numel(criticalIDs))); end
end
affectedIDs=intersect(affected,activeIDs,'stable');
info.previousRoute=oldRoute;
info.criticalIDs=criticalIDs;
info.affectedCount=numel(affectedIDs);
info.activeCount=numel(activeIDs);
info.affectedRatio=numel(affectedIDs)/max(numel(activeIDs),1);
end
