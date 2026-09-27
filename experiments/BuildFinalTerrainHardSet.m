function BuildFinalTerrainHardSet
%BUILDFINALTERRAINHARDSET 构造用于最终动态边界验证的M=4窄窗实例
%   通过多订单新增+窄时间窗使warm-start不再天然等价于可行解，
%   同时保留一个由完整路由搜索得到的可行witness。
root=fileparts(fileparts(mfilename('fullpath'))); setup;
outDir=fullfile(root,'results','final_local_terrain_hard_instances');
if ~isfolder(outDir), mkdir(outDir); end
[previous,baseModel,~]=BuildPreEventSolution('local-terrain');
baseModel.windows(:,2)=baseModel.windows(:,2)+1000;
baseModel.cfg.silent=true; activeInitial=baseModel.customerIDs(:)';
baseCache=BuildLegCache(baseModel,activeInitial,baseModel.homePosition);
files=cell(8,1); rows=cell(0,1); eventTime=90; M=4; slack=8;
for s=1:8
    scenarioSeed=910000+s; rng(scenarioSeed,'twister');
    state=ExecuteUntilEvent(previous,baseModel,eventTime);
    activeIDs=setdiff(activeInitial,state.completedIDs,'stable');
    model=baseModel; model.startTime=eventTime; model.depot=state.position;
    customers=repmat(struct('id',0,'xy',[0 0],'window',[0 2000],'service',10),M,1);
    for k=1:M
        if mod(s,2)==1
            xy=FarPoint(model,activeIDs,180);
        else
            route=previous.Detail.routeIDs(ismember(previous.Detail.routeIDs,activeIDs));
            pos=randi(numel(route)+1); xy=SegmentPoint(route,pos,model,state);
        end
        id=baseModel.nCustomers+k;
        customers(k).id=id; customers(k).xy=xy; customers(k).service=8+4*rand;
        model=AddCustomer(model,customers(k)); activeIDs=[activeIDs,id]; %#ok<AGROW>
    end
    cache=BuildLegCache(model,activeIDs,state.position);
    % 宽窗阶段只用于构造可行路线；随后再把新增订单窗口收紧。
    [referenceSolution,~,~]=RoutingPSO(model,cache,220,40,0.90,0.995,1.7,1.7,[]);
    if ~referenceSolution.Detail.isFeasible
        error('Hard witness failed for scenario %d.',s);
    end
    route=referenceSolution.Detail.routeIDs; [referenceCost,detail]=EvaluateSchedule(route,model,cache);
    for k=1:M
        id=customers(k).id; hit=find(detail.records(:,1)==id,1);
        serviceStart=detail.records(hit,3);
        customers(k).window=[max(eventTime,serviceStart-slack),serviceStart+slack];
        model.windows(id,:)=customers(k).window;
    end
    [referenceCost,detail]=EvaluateSchedule(route,model,cache);
    if ~detail.isFeasible, error('Tightened witness failed for scenario %d.',s); end
    event.type='add'; event.time=eventTime; event.customerIDs=[customers.id]; event.customers=customers;
    instance.level='M4-tight'; instance.instance=s; instance.scenarioSeed=scenarioSeed;
    instance.event=event; instance.state=state; instance.activeIDs=activeIDs;
    instance.referenceRoute=route; instance.referenceSchedule=detail.records;
    instance.referenceCost=referenceCost; instance.model=model; instance.cache=cache;
    fileName=sprintf('hard_M4_%02d.mat',s); files{s}=fileName;
    save(fullfile(outDir,fileName),'instance');
    rows{end+1,1}={fileName,scenarioSeed,referenceCost,numel(activeIDs),slack}; %#ok<AGROW>
    fprintf('%s generated.\n',fileName);
end
manifest=cell2table(vertcat(rows{:}),'VariableNames', ...
    {'file_name','scenario_seed','reference_cost','active_customers','window_slack'});
writetable(manifest,fullfile(outDir,'hard_manifest.csv')); disp(manifest);
end

function xy=FarPoint(model,activeIDs,minDistance)
for t=1:1000
    xy=[40+920*rand,40+720*rand];
    if min(sqrt(sum((model.customerXY(activeIDs,:)-xy).^2,2)))>minDistance, return; end
end
xy=[950 760];
end
function xy=SegmentPoint(route,pos,model,state)
if pos==1, from=state.position; else, from=model.customerXYZ(route(pos-1),:); end
if pos>numel(route), to=model.homePosition; else, to=model.customerXYZ(route(pos),:); end
delta=to(1:2)-from(1:2); n=norm(delta);
if n<eps, p=[0 0]; else, p=[-delta(2),delta(1)]/n; end
xy=from(1:2)+0.5*delta+0.08*n*(2*rand-1)*p;
xy(1)=min(max(xy(1),20),980); xy(2)=min(max(xy(2),20),780);
end
function model=AddCustomer(model,customer)
groundZ=interp2(model.X,model.Y,model.terrainZ,customer.xy(1),customer.xy(2),'linear');
id=customer.id; model.customerXY(id,:)=customer.xy;
model.customerXYZ(id,:)=[customer.xy,groundZ+model.cruiseHeight];
model.customerGroundXYZ(id,:)=[customer.xy,groundZ];
model=UpdateLocalCruiseAltitudes(model); model.windows(id,:)=customer.window;
model.service(id,1)=customer.service; model.customerIDs=(1:size(model.customerXY,1))';
model.nCustomers=numel(model.customerIDs);
end
