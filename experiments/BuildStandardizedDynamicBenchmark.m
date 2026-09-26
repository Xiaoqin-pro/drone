function BuildStandardizedDynamicBenchmark
%BUILDSTANDARDIZEDDYNAMICBENCHMARK 生成可行的标准Solomon单UAV子实例
scriptDir=fileparts(mfilename('fullpath')); root=fileparts(scriptDir);
rawDir=fullfile(root,'data','solomon_raw','selected'); outDir=fullfile(root,'data','standardized_dynamic');
if ~exist(outDir,'dir'), mkdir(outDir); end
names={'r101','c101','rc101'}; target=20; revealFraction=0.20;
for s=1:numel(names)
    solomon=ReadSolomonInstance(fullfile(rawDir,[names{s} '.txt']));
    rng(4100+s,'twister');
    [selectedRows,preRoute,preSchedule]=SelectFeasibleSingleRoute(solomon,target);
    base=CreateModel(); model=ConvertToSingleUAVInstance(solomon,base,selectedRows,false);
    n=model.nCustomers;
    finishTime=preSchedule(end,2)+solomon.customers(preRoute(end),7);
    localRoute=zeros(size(preRoute));
    for q=1:numel(preRoute), localRoute(q)=find(selectedRows==preRoute(q),1); end
    eventTime=0.5*finishTime;
    hiddenSource=preRoute(preSchedule(:,2)>eventTime & solomon.customers(preRoute,6)>eventTime);
    hiddenCount=min(max(1,round(n*revealFraction)),numel(hiddenSource));
    hiddenSource=hiddenSource(max(1,end-hiddenCount+1):end);
    hiddenIDs=zeros(size(hiddenSource));
    for q=1:numel(hiddenSource), hiddenIDs(q)=find(selectedRows==hiddenSource(q),1); end
    activeIDs=setdiff(model.customerIDs,hiddenIDs,'stable');
    event.time=eventTime; event.type='add'; event.customerIDs=hiddenIDs;
    for k=1:numel(hiddenIDs)
        id=hiddenIDs(k); event.customers(k).id=id; %#ok<AGROW>
        event.customers(k).xy=model.customerXY(id,:); %#ok<AGROW>
        event.customers(k).window=model.windows(id,:); %#ok<AGROW>
        event.customers(k).service=model.service(id); %#ok<AGROW>
    end
    instance.sourceBenchmark=names{s}; instance.sourceName=solomon.name;
    instance.sourceCustomerIDs=solomon.customers(selectedRows,1);
    instance.customerCount=n; instance.selectedRows=selectedRows;
    instance.model=model; instance.activeIDs=activeIDs; instance.hiddenIDs=hiddenIDs;
    instance.event=event; instance.preEventRoute=localRoute; instance.preEventSchedule=preSchedule;
    instance.preEventFinishTime=finishTime; instance.admitted=true; instance.use3D=false;
    save(fullfile(outDir,[names{s} '_20_dynamic.mat']),'instance');
    fprintf('%s: selected %d feasible single-route customers; active=%d hidden=%d.\n', ...
        names{s},n,numel(activeIDs),numel(hiddenIDs));
end
end

function [selectedRows,route,schedule]=SelectFeasibleSingleRoute(solomon,target)
allRows=solomon.customers; remaining=(1:size(allRows,1))'; selectedRows=zeros(0,1); route=zeros(1,0);
while ~isempty(remaining) && numel(selectedRows)<target
    order=remaining(randperm(numel(remaining)));
    inserted=false;
    for idx=order'
        bestRoute=[]; bestSchedule=[];
        for pos=1:numel(route)+1
            candidate=[route(1:pos-1),idx,route(pos:end)];
            [ok,sc]=EvaluateSimpleRoute(candidate,allRows,solomon.depotRow);
            if ok, bestRoute=candidate; bestSchedule=sc; break; end
        end
        if ~isempty(bestRoute)
            route=bestRoute; selectedRows=unique([selectedRows;idx],'stable'); schedule=bestSchedule; remaining(remaining==idx)=[]; inserted=true; break;
        end
    end
    if ~inserted, break; end
end
if isempty(route), error('No feasible single-UAV subset found for %s.',solomon.name); end
[~,schedule]=EvaluateSimpleRoute(route,allRows,solomon.depotRow);
end

function [ok,schedule]=EvaluateSimpleRoute(route,rows,depot)
current=[depot(2),depot(3)]; time=0; schedule=zeros(numel(route),3); ok=true;
for k=1:numel(route)
    row=rows(route(k),:); distance=norm(current-row(2:3)); arrival=time+distance; start=max(arrival,row(5));
    if start>row(6)+1e-9, ok=false; return; end
    schedule(k,:)=[route(k),start,distance]; time=start+row(7); current=row(2:3);
end
end
