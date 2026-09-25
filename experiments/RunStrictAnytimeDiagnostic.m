function RunStrictAnytimeDiagnostic
%RUNSTRICTANYTIMEDIAGNOSTIC 一次运行到2500FE并保存真实checkpoint
scriptDir=fileparts(mfilename('fullpath'));
root=fileparts(scriptDir);
resultDir=fullfile(root,'results');
checkpoints=[500 1000 1500 2500];
strategies={'Restart','WarmStart','Repair'};
rows=cell(0,1);

datasets={'addition_instances','addition_stress_instances'};
for d=1:numel(datasets)
    dataDir=fullfile(resultDir,datasets{d});
    mf=dir(fullfile(dataDir,'*manifest.csv'));
    if isempty(mf), continue; end
    manifest=readtable(fullfile(dataDir,mf(1).name),'TextType','string');
    if ismember('level',manifest.Properties.VariableNames)
        levelColumn=string(manifest.level);
    else
        levelColumn=string(manifest.stress_level);
    end
    for r=1:height(manifest)
        loaded=load(fullfile(dataDir,manifest.file_name(r)));
        instance=loaded.instance;
        baseModel=CreateModel();
        baseModel.windows(:,2)=baseModel.windows(:,2)+300;
        baseModel.cfg.silent=true;
        ids=baseModel.customerIDs(:)';
        baseCache=BuildLegCache(baseModel,ids,baseModel.homePosition);
        rng(860000,'twister');
        [initialSolution,~,~]=RoutingPSO(baseModel,baseCache,100,40, ...
            0.90,0.995,1.7,1.7,[]);
        model=instance.model;
        model.cfg.silent=true;
        cache=instance.cache;
        activeIDs=instance.activeIDs;
        event=instance.event;
        bksCost=instance.referenceCost;
        for si=1:numel(strategies)
            strategy=strategies{si};
            seed=990000+d*10000+r*100+si;
            warmFE=0;
            warmTime=0;
            if strcmpi(strategy,'WarmStart')
                rng(seed,'twister');
                t=tic;
                [initialPositions,warmInfo]=BuildWarmStartPopulation( ...
                    initialSolution,activeIDs,model,cache,30);
                warmTime=toc(t);
                warmFE=warmInfo.evaluations;
            else
                initialPositions=[];
            end
            if strcmpi(strategy,'Repair')
                t=tic;
                [solution,repairInfo]=LocalRepair(initialSolution,activeIDs, ...
                    model,cache,event.type,event.customerIDs);
                routingTime=toc(t);
                routingHistory=[];
                routeFE=repairInfo.functionEvaluations;
            else
                available=max(0,2500-warmFE);
                maxIt=max(0,floor(available/30)-1);
                rng(seed,'twister');
                t=tic;
                [solution,~,stats,history]=RoutingPSO(model,cache,maxIt,30, ...
                    0.90,0.995,1.7,1.7,initialPositions);
                routingTime=toc(t);
                routingHistory=history;
                routeFE=stats.functionEvaluations;
            end

            for cp=checkpoints
                if strcmpi(strategy,'Repair')
                    cost=solution.Cost;
                    feasible=solution.Detail.isFeasible;
                    actualFE=routeFE;
                else
                    index=find(routingHistory.FE+warmFE<=cp,1,'last');
                    if isempty(index)
                        continue;
                    end
                    cost=routingHistory.Cost(index);
                    feasible=routingHistory.IsFeasible(index);
                    actualFE=warmFE+routingHistory.FE(index);
                end
                rows{end+1,1}={string(datasets{d}),levelColumn(r), ...
                    manifest.instance(r),string(strategy),seed,cp,actualFE, ...
                    cost,(cost-bksCost)/max(abs(bksCost),eps), ...
                    feasible,warmFE,routingTime+warmTime}; %#ok<AGROW>
            end
        end
    end
end

summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'dataset','level','instance','strategy','algorithm_seed','checkpoint_fe', ...
    'actual_fe','cost','gap_to_witness_bks','is_feasible','warm_start_fe', ...
    'response_time'});
writetable(summary,fullfile(resultDir,'strict_anytime_summary.csv'));
save(fullfile(resultDir,'strict_anytime_result.mat'),'summary','checkpoints');
end
