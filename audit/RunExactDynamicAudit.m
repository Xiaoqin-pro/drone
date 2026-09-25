function RunExactDynamicAudit
%RUNEXACTDYNAMICAUDIT 用MILP审计事件发生后的上层TSPTW可行性

clc
clear
close all

root = fileparts(mfilename('fullpath'));
levels = {'Mild','Moderate','Severe'};
nInstances = 2;
scenarioSeedBase = 3000;
maxTime = 60;
outputDir = fullfile(root,'results');
rows = cell(0,1);

for levelIndex = 1:numel(levels)
    for instanceIndex = 1:nInstances
        scenarioSeed = scenarioSeedBase+instanceIndex;
        rng(scenarioSeed,'twister');
        baseModel = CreateModel();
        instance = GenerateDynamicInstance(baseModel,scenarioSeed, ...
            levels{levelIndex});
        activeIDs = baseModel.customerIDs(:)';
        model = baseModel;
        cache = BuildLegCache(model,activeIDs,model.homePosition);
        rng(880000+instanceIndex,'twister');
        [solution,~,~] = RoutingPSO(model,cache,50,30, ...
            0.90,0.995,1.7,1.7,[]);

        for e = 1:numel(instance.events)
            event = instance.events(e);
            state = ExecuteUntilEvent(solution,model,event.time);
            activeIDs = setdiff(activeIDs,state.completedIDs,'stable');
            model = ApplyReferenceEvent(model,event);
            if strcmpi(event.type,'cancel')
                activeIDs = setdiff(activeIDs,event.customerIDs,'stable');
            else
                activeIDs = [activeIDs,event.customerIDs]; %#ok<AGROW>
            end
            model.startTime = event.time;
            model.depot = state.position;
            [cache,cacheStats] = UpdateLegCache(model,cache, ...
                activeIDs,state.position);

            oracle = ExactTWFeasibilityOracle(model,cache,activeIDs,maxTime);
            rows{end+1,1} = {string(levels{levelIndex}),instanceIndex, ...
                scenarioSeed,e,event.time,string(event.type), ...
                numel(event.customerIDs),oracle.status,oracle.exitflag, ...
                oracle.objective,oracle.nEdges,cacheStats.reusedCount, ...
                cacheStats.computedCount,state.position(1), ...
                state.position(2),state.position(3),state.time}; %#ok<AGROW>

            fprintf('%s instance %d event %d: %s, obj=%.2f, edges=%d\n', ...
                levels{levelIndex},instanceIndex,e,oracle.status, ...
                oracle.objective,oracle.nEdges);
            if strcmp(oracle.status,'FEASIBLE')
                solution = SolutionFromRoute(oracle.referenceRoute,model,cache);
            else
                break;
            end
        end
    end
end

summary = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'level','instance','scenario_seed','event_index','event_time', ...
    'event_type','event_size','oracle_status','exitflag','reference_cost', ...
    'allowed_edges','reused_legs','computed_legs','state_x','state_y', ...
    'state_z','state_time'});
writetable(summary,fullfile(outputDir,'exact_dynamic_audit.csv'));
save(fullfile(outputDir,'exact_dynamic_audit.mat'), ...
    'summary','levels','nInstances','maxTime');
end

function model = ApplyReferenceEvent(model,event)
if strcmpi(event.type,'add')
    for k = 1:numel(event.customers)
        customer = event.customers(k);
        id = customer.id;
        groundZ = interp2(model.X,model.Y,model.terrainZ, ...
            customer.xy(1),customer.xy(2),'linear');
        model.customerXY(id,:) = customer.xy;
        model.customerXYZ(id,:) = [customer.xy,model.flightAltitude];
        model.customerGroundXYZ(id,:) = [customer.xy,groundZ];
        model.windows(id,:) = customer.window;
        model.service(id,1) = customer.service;
    end
end
model.customerIDs = (1:size(model.customerXY,1))';
model.nCustomers = numel(model.customerIDs);
end

function solution = SolutionFromRoute(routeIDs,model,cache)
[cost,detail] = EvaluateSchedule(routeIDs,model,cache);
solution.Cost = cost;
solution.Detail = detail;
solution.Route = detail.routeIDs;
end


