function RunStressAdditionPaired
%RUNSTRESSADDITIONPAIRED 比较三策略在Near/Far/Tight新增事件上的表现
scriptDir = fileparts(mfilename('fullpath'));
root = fileparts(scriptDir);
manifest = readtable(fullfile(root,'results','addition_stress_instances', ...
    'stress_manifest.csv'),'TextType','string');
levels = unique(string(manifest.stress_level),'stable');
strategies = {'Restart','WarmStart','Repair'};
budgets = [500 1000 1500 2500];
nInstancesPerLevel = 5;
cfg.population = 30;
cfg.inertia = 0.90;
cfg.inertiaDamp = 0.995;
cfg.c1 = 1.7;
cfg.c2 = 1.7;
cfg.outputDir = fullfile(root,'results');
rows = cell(0,1);

for li=1:numel(levels)
    levelRows=manifest(string(manifest.stress_level)==levels(li),:);
    levelRows=levelRows(1:min(nInstancesPerLevel,height(levelRows)),:);
    for r=1:height(levelRows)
        loaded=load(fullfile(root,'results','addition_stress_instances', ...
            levelRows.file_name(r)));
        instance=loaded.instance;
        baseModel=CreateModel();
        baseModel.windows(:,2)=baseModel.windows(:,2)+300;
        baseModel.cfg.silent=true;
        ids=baseModel.customerIDs(:)';
        baseCache=BuildLegCache(baseModel,ids,baseModel.homePosition);
        rng(860000,'twister');
        [initialSolution,~,~]=RoutingPSO(baseModel,baseCache,100,40, ...
            cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2,[]);
        model=instance.model; cache=instance.cache; activeIDs=instance.activeIDs;
        event=instance.event; referenceCost=instance.referenceCost;
        for si=1:numel(strategies)
            for bi=1:numel(budgets)
                strategy=strategies{si}; budget=budgets(bi);
                seed=9700+100*r+budget;
                [solution,metrics]=RunStrategy(strategy,initialSolution, ...
                    model,cache,activeIDs,event,budget,cfg,seed);
                gap=(solution.Cost-referenceCost)/max(abs(referenceCost),eps);
                rows{end+1,1}={levels(li),r,instance.scenarioSeed, ...
                    string(strategy),budget,metrics.warmFE,metrics.routeFE, ...
                    metrics.totalFE,metrics.time,solution.Cost, ...
                    solution.Detail.distance,solution.Detail.totalLate, ...
                    solution.Detail.isFeasible,gap, ...
                    ComputeRouteDisruption(initialSolution.Detail.routeIDs, ...
                    solution.Detail.routeIDs)}; %#ok<AGROW>
            end
        end
        fprintf('%s stress instance %d/%d completed.\n',levels(li),r,nInstancesPerLevel);
    end
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'stress_level','instance','scenario_seed','strategy','budget', ...
    'warm_start_fe','routing_fe','total_fe','response_time','fitness', ...
    'distance','total_late','is_feasible','gap_to_reference', ...
    'route_disruption'});
writetable(summary,fullfile(cfg.outputDir,'stress_addition_summary.csv'));
save(fullfile(cfg.outputDir,'stress_addition_result.mat'),'summary','cfg');
end

function [solution,m] = RunStrategy(strategy,previous,model,cache,ids,event,budget,cfg,seed)
m.warmFE=0; m.time=0;
if strcmpi(strategy,'Repair')
    t=tic; [solution,info]=LocalRepair(previous,ids,model,cache,event.type,event.customerIDs); m.time=toc(t); m.routeFE=info.functionEvaluations;
else
    if strcmpi(strategy,'WarmStart')
        rng(seed,'twister'); t=tic; [pos,wi]=BuildWarmStartPopulation(previous,ids,model,cache,cfg.population); m.warmFE=wi.evaluations; m.warmTime=toc(t);
    else
        pos=[]; m.warmTime=0;
    end
    available=max(0,budget-m.warmFE); maxIt=max(0,floor(available/cfg.population)-1); rng(seed,'twister'); t=tic;
    [solution,~,st]=RoutingPSO(model,cache,maxIt,cfg.population,cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2,pos); m.time=m.warmTime+toc(t); m.routeFE=st.functionEvaluations;
end
m.totalFE=m.warmFE+m.routeFE;
end
