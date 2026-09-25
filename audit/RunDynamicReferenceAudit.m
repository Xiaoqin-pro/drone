function RunDynamicReferenceAudit
%RUNDYNAMICREFERENCEAUDIT 为动态事件状态生成参考可行解候选
%   该脚本先用统一参考路线执行事件，再用较大预算搜索剩余任务。

clc
clear
close all

root = fileparts(mfilename('fullpath'));
levels = {'Mild','Moderate','Severe'};
nInstances = 2; % 先验收；确认后改为10
scenarioSeedBase = 3000;
referenceBudget = 1200;
referenceRestarts = 1;
outputDir = fullfile(root,'results');

rows = cell(0,1);
allStates = cell(0,1);

for levelIndex = 1:numel(levels)
    for instanceIndex = 1:nInstances
        scenarioSeed = scenarioSeedBase+instanceIndex;
        rng(scenarioSeed,'twister');
        baseModel = CreateModel();
        baseModel.cfg.silent = true;
        instance = GenerateDynamicInstance(baseModel,scenarioSeed, ...
            levels{levelIndex});
        activeIDs = baseModel.customerIDs(:)';
        cache = BuildLegCache(baseModel,activeIDs,baseModel.homePosition);

        rng(880000+instanceIndex,'twister');
        [initialSolution,~,~] = RoutingPSO(baseModel,cache,50,30, ...
            0.90,0.995,1.7,1.7,[]);

        model = baseModel;
        solution = initialSolution;
        for e = 1:numel(instance.events)
            event = instance.events(e);
            state = ExecuteUntilEvent(solution,model,event.time);
            activeIDs = setdiff(activeIDs,state.completedIDs,'stable');
            model = ApplyReferenceEvent(model,event,activeIDs);
            if strcmpi(event.type,'cancel')
                activeIDs = setdiff(activeIDs,event.customerIDs,'stable');
            else
                activeIDs = [activeIDs,event.customerIDs]; %#ok<AGROW>
            end
            model.startTime = event.time;
            model.depot = state.position;
            [cache,cacheStats] = UpdateLegCache(model,cache, ...
                activeIDs,state.position);

            report = BuildReferenceSolution(model,cache,activeIDs, ...
                referenceBudget,referenceRestarts);
            rows{end+1,1} = {string(levels{levelIndex}), ...
                instanceIndex,scenarioSeed,e,event.time, ...
                string(event.type),numel(event.customerIDs), ...
                state.position(1),state.position(2),state.position(3), ...
                state.time,numel(state.completedIDs),cacheStats.reusedCount, ...
                cacheStats.computedCount,report.isFound, ...
                report.referenceCost,report.totalFE, ...
                report.referenceStats.firstFeasibleEvaluation}; %#ok<AGROW>
            allStates{end+1,1} = struct('level',levels{levelIndex}, ...
                'instance',instanceIndex,'event',e,'state',state, ...
                'activeIDs',activeIDs,'model',model,'cache',cache, ...
                'report',report); %#ok<AGROW>

            fprintf('%s instance %d event %d: found=%d, cost=%.2f, FE=%d\n', ...
                levels{levelIndex},instanceIndex,e,report.isFound, ...
                report.referenceCost,report.totalFE);
            if report.isFound
                solution = report.referenceSolution;
            else
                break;
            end
        end
    end
end

summary = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'level','instance','scenario_seed','event_index','event_time', ...
    'event_type','event_size','state_x','state_y','state_z','state_time', ...
    'completed_count','reused_legs','computed_legs','reference_found', ...
    'reference_cost','reference_fe','reference_first_fe'});
writetable(summary,fullfile(outputDir,'dynamic_reference_audit.csv'));
save(fullfile(outputDir,'dynamic_reference_audit.mat'), ...
    'summary','allStates','levels','nInstances','referenceBudget');
end

function model = ApplyReferenceEvent(model,event,activeIDs)
if strcmpi(event.type,'add')
    for k = 1:numel(event.customers)
        customer = event.customers(k);
        groundZ = interp2(model.X,model.Y,model.terrainZ, ...
            customer.xy(1),customer.xy(2),'linear');
        id = customer.id;
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
