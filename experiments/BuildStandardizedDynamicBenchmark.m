function BuildStandardizedDynamicBenchmark
%BUILDSTANDARDIZEDDYNAMICBENCHMARK 生成可行的标准Solomon单UAV动态子实例
scriptDir=fileparts(mfilename('fullpath')); root=fileparts(scriptDir);
rawDir=fullfile(root,'data','solomon_raw','selected'); outDir=fullfile(root,'data','standardized_dynamic');
if ~exist(outDir,'dir'), mkdir(outDir); end
names={'r101','c101','rc101'}; target=20; revealFraction=0.20;
for s=1:numel(names)
    solomon=ReadSolomonInstance(fullfile(rawDir,[names{s} '.txt']));
    rng(4100+s,'twister');
    [selectedRows,sourceRoute,~]=SelectFeasibleSingleRoute(solomon,target);
    base=CreateModel(); model=ConvertToSingleUAVInstance(solomon,base,selectedRows,false);
    n=model.nCustomers; localRoute=zeros(size(sourceRoute));
    for q=1:numel(sourceRoute), localRoute(q)=find(selectedRows==sourceRoute(q),1); end
    hiddenCount=max(1,round(n*revealFraction));
    [hiddenIDs,eventTime,state,preActiveRoute,postRoute,preDetail,postDetail]=FindFeasibleReveal(model,localRoute,hiddenCount);
    activeIDs=setdiff(model.customerIDs,hiddenIDs,'stable');
    event.time=eventTime; event.type='add'; event.customerIDs=hiddenIDs;
    for k=1:numel(hiddenIDs)
        id=hiddenIDs(k); event.customers(k).id=id; event.customers(k).xy=model.customerXY(id,:); event.customers(k).window=model.windows(id,:); event.customers(k).service=model.service(id);
    end
    instance.sourceBenchmark=names{s}; instance.sourceName=solomon.name; instance.sourceCustomerIDs=solomon.customers(selectedRows,1);
    instance.customerCount=n; instance.selectedRows=selectedRows; instance.model=model; instance.activeIDs=activeIDs; instance.hiddenIDs=hiddenIDs; instance.event=event;
    instance.preEventRoute=preActiveRoute; instance.preEventSchedule=preDetail.records; instance.preEventState=state; instance.postWitnessRoute=postRoute; instance.postWitnessSchedule=postDetail.records; instance.preEventFinishTime=preDetail.finishTime; instance.admitted=true; instance.use3D=false;
    save(fullfile(outDir,[names{s} '_20_dynamic.mat']),'instance');
    fprintf('%s: selected %d feasible single-route customers; active=%d hidden=%d.\n',names{s},n,numel(activeIDs),numel(hiddenIDs));
end
end
function [hiddenIDs,eventTime,state,preRoute,postRoute,preDetail,postDetail]=FindFeasibleReveal(model,route,hiddenCount)
n=numel(route); fractions=[0.35 0.50 0.65]; found=false;
for frac=fractions
    for start=1:max(1,n-hiddenCount+1)
        hiddenIDs=route(start:min(n,start+hiddenCount-1)); activeIDs=setdiff(model.customerIDs,hiddenIDs,'stable'); preRoute=route(ismember(route,activeIDs));
        preModel=model; preModel.startTime=0; preModel.depot=model.homePosition; preCache=BuildLegCache(preModel,activeIDs,model.homePosition); [~,preDetail]=EvaluateSchedule(preRoute,preModel,preCache); if ~preDetail.isFeasible, continue; end
        eventTime=frac*preDetail.finishTime; sol.Cost=preDetail.distance; sol.Detail=preDetail; sol.Route=preRoute; state=ExecuteUntilEvent(sol,preModel,eventTime); if ~all(model.windows(hiddenIDs,2)>eventTime), continue; end
        postModel=model; postModel.startTime=eventTime; postModel.depot=state.position; allCache=BuildLegCache(postModel,model.customerIDs,state.position); postRoute=route(~ismember(route,state.completedIDs)); [~,postDetail]=EvaluateSchedule(postRoute,postModel,allCache); if postDetail.isFeasible, found=true; return; end
    end
end
if ~found, error('No feasible dynamic reveal configuration found.'); end
end
function [selectedRows,route,schedule]=SelectFeasibleSingleRoute(solomon,target)
allRows=solomon.customers; remaining=(1:size(allRows,1))'; selectedRows=zeros(0,1); route=zeros(1,0);
while ~isempty(remaining) && numel(selectedRows)<target
    order=remaining(randperm(numel(remaining))); inserted=false;
    for idx=order'
        for pos=1:numel(route)+1
            candidate=[route(1:pos-1),idx,route(pos:end)]; [ok,sc]=EvaluateSimpleRoute(candidate,allRows,solomon.depotRow);
            if ok, route=candidate; selectedRows=unique([selectedRows;idx],'stable'); schedule=sc; remaining(remaining==idx)=[]; inserted=true; break; end
        end
        if inserted, break; end
    end
    if ~inserted, break; end
end
if isempty(route), error('No feasible single-UAV subset found for %s.',solomon.name); end
[~,schedule]=EvaluateSimpleRoute(route,allRows,solomon.depotRow);
end
function [ok,schedule]=EvaluateSimpleRoute(route,rows,depot)
current=[depot(2),depot(3)]; time=0; schedule=zeros(numel(route),3); ok=true;
for k=1:numel(route)
    row=rows(route(k),:); distance=norm(current-row(2:3)); arrival=time+distance; start=max(arrival,row(5)); if start>row(6)+1e-9, ok=false; return; end
    schedule(k,:)=[route(k),start,distance]; time=start+row(7); current=row(2:3);
end
end
