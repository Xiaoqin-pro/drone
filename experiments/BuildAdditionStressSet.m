function BuildAdditionStressSet
%BUILDADDITIONSTRESSSET 构造新增订单冲击强度诊断实例
%   用于检验Local Repair是否只在“近距离可行插入”场景中占优。

clc
clear
close all

scriptDir = fileparts(mfilename('fullpath'));
root = fileparts(scriptDir);
levels = {'Near','Far','Tight'};
nInstances = 5;
seedBase = 8000;
outputDir = fullfile(root,'results','addition_stress_instances');
if ~exist(outputDir,'dir')
    mkdir(outputDir);
end
rows = cell(0,1);

for levelIndex = 1:numel(levels)
    for instanceIndex = 1:nInstances
        scenarioSeed = seedBase+100*levelIndex+instanceIndex;
        rng(scenarioSeed,'twister');
        baseModel = CreateModel();
        baseModel.windows(:,2) = baseModel.windows(:,2)+300;
        baseModel.cfg.silent = true;
        activeInitial = baseModel.customerIDs(:)';
        baseCache = BuildLegCache(baseModel,activeInitial,baseModel.homePosition);
        scenarioState = rng;
        rng(860000,'twister');
        [baseSolution,~,~] = RoutingPSO(baseModel,baseCache,100,40, ...
            0.90,0.995,1.7,1.7,[]);
        rng(scenarioState);
        state = ExecuteUntilEvent(baseSolution,baseModel,90);
        activeIDs = setdiff(activeInitial,state.completedIDs,'stable');
        model = baseModel;
        model.startTime = 90;
        model.depot = state.position;
        route = baseSolution.Detail.routeIDs(ismember(...
            baseSolution.Detail.routeIDs,activeIDs));
        M = 2;
        baseCustomerCount = model.nCustomers;
        customers = repmat(struct('id',0,'xy',[0 0], ...
            'window',[0 0],'service',10),M,1);
        for k = 1:M
            if strcmp(levels{levelIndex},'Far')
                xy = RandomFarPoint(model,activeIDs);
                slack = 80;
            else
                pos = randi(numel(route)+1);
                xy = SegmentPoint(route,pos,model,state);
                if strcmp(levels{levelIndex},'Tight')
                    slack = 25;
                else
                    slack = 80;
                end
            end
            id = baseCustomerCount+k;
            customers(k).id = id;
            customers(k).xy = xy;
            customers(k).window = [0 2000];
            customers(k).service = 10;
            model = AddCustomer(model,customers(k));
            activeIDs = [activeIDs,id]; %#ok<AGROW>
            route = InsertAt(route,id,randi(numel(route)+1));
        end
        cache = BuildLegCache(model,activeIDs,state.position);
        [~,detail] = EvaluateSchedule(route,model,cache);
        if ~detail.isFeasible
            [referenceSolution,~,~] = RoutingPSO(model,cache,160,40, ...
                0.90,0.995,1.7,1.7,[]);
            if ~referenceSolution.Detail.isFeasible
                error('Stress witness failed: %s %d',levels{levelIndex},instanceIndex);
            end
            route = referenceSolution.Detail.routeIDs;
            detail = referenceSolution.Detail;
        end
        for k = 1:M
            id = customers(k).id;
            hit = find(detail.records(:,1)==id,1);
            serviceStart = detail.records(hit,3);
            customers(k).window = [max(0,serviceStart-slack), ...
                serviceStart+slack];
            model.windows(id,:) = customers(k).window;
        end
        [referenceCost,detail] = EvaluateSchedule(route,model,cache);
        if ~detail.isFeasible
            error('Stress window tightening failed: %s %d', ...
                levels{levelIndex},instanceIndex);
        end
        event.type = 'add'; event.time = 90;
        event.customerIDs = [customers.id]; event.customers = customers;
        instance.level = levels{levelIndex};
        instance.instance = instanceIndex;
        instance.scenarioSeed = scenarioSeed;
        instance.event = event;
        instance.state = state;
        instance.activeIDs = activeIDs;
        instance.referenceRoute = route;
        instance.referenceCost = referenceCost;
        instance.model = model;
        instance.cache = cache;
        fileName = sprintf('stress_%s_%02d.mat',levels{levelIndex},instanceIndex);
        save(fullfile(outputDir,fileName),'instance');
        rows{end+1,1} = {string(levels{levelIndex}),instanceIndex, ...
            scenarioSeed,fileName,referenceCost}; %#ok<AGROW>
    end
    fprintf('%s stress instances generated.\n',levels{levelIndex});
end
manifest = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'stress_level','instance','scenario_seed','file_name','reference_cost'});
writetable(manifest,fullfile(outputDir,'stress_manifest.csv'));
disp(manifest);
end

function xy = SegmentPoint(route,pos,model,state)
if pos==1, from=state.position; else, from=model.customerXYZ(route(pos-1),:); end
if pos>numel(route), to=model.homePosition; else, to=model.customerXYZ(route(pos),:); end
delta=to(1:2)-from(1:2); n=norm(delta);
if n<eps, p=[0 0]; else, p=[-delta(2),delta(1)]/n; end
xy=from(1:2)+0.5*delta+0.08*n*(2*rand-1)*p;
xy(1)=min(max(xy(1),20),980); xy(2)=min(max(xy(2),20),780);
end

function xy = RandomFarPoint(model,activeIDs)
for k=1:1000
    xy=[50+900*rand,50+700*rand];
    points=model.customerXY(activeIDs,:);
    if min(sqrt(sum((points-xy).^2,2)))>220
        return;
    end
end
xy=[900 750];
end

function model=AddCustomer(model,customer)
groundZ=interp2(model.X,model.Y,model.terrainZ,customer.xy(1),customer.xy(2),'linear');
id=customer.id;
model.customerXY(id,:)=customer.xy;
model.customerXYZ(id,:)=[customer.xy,model.flightAltitude];
model.customerGroundXYZ(id,:)=[customer.xy,groundZ];
model.windows(id,:)=customer.window;
model.service(id,1)=customer.service;
model.customerIDs=(1:size(model.customerXY,1))';
model.nCustomers=numel(model.customerIDs);
end

function route=InsertAt(route,id,pos)
route=[route(1:pos-1),id,route(pos:end)];
end
