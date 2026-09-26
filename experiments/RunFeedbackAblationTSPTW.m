function results = RunFeedbackAblationTSPTW
%RUNFEEDBACKABLATIONTSPTW 检验离散Relocate经验是否能反向增强PSO

root=fileparts(fileparts(mfilename('fullpath'))); setup;
benchmarkDir=fullfile(root,'data','tsp_tw_benchmark');
manifest=readtable(fullfile(benchmarkDir,'manifest.csv'),'Delimiter',',','TextType','string');
reference=ReadTSPTWReferenceSolutions(fullfile(root,'data','tsp_tw_raw','best_known', ...
    'SolomonPotvinBengio-best-known-traveltime.txt'));
methods=["PSO","FixedRelocate","StateSwitch","Feedback","Full"];
seeds=1:10; factors=[25 50 100]; rows=cell(0,1);
for i=1:height(manifest)
    instance=ReadTSPTWInstance(fullfile(benchmarkDir,manifest.file(i)));
    refIndex=find(strcmpi(string({reference.file}),manifest.file(i)),1); refCost=reference(refIndex).cost;
    for seedIndex=1:numel(seeds)
        seed=980000+100*i+seeds(seedIndex);
        for factor=factors
            budget=factor*instance.nCustomers;
            for m=1:numel(methods)
                rng(seed,'twister'); options=struct('nPop',20,'maxFE',budget, ...
                    'latePenalty',1000,'localSearchFE',instance.nCustomers, ...
                    'localSearchMode','none','feedbackEnabled',false, ...
                    'feedbackLearningRate',0.35,'feedbackDecay',0.80,'silent',true);
                switch methods(m)
                    case "FixedRelocate", options.localSearchMode='global';
                    case "StateSwitch", options.localSearchMode='state-switch';
                    case "Feedback", options.localSearchMode='global'; options.feedbackEnabled=true;
                    case "Full", options.localSearchMode='state-switch'; options.feedbackEnabled=true;
                    otherwise, options.localSearchFE=0;
                end
                [best,history,stats]=RandomKeyTSPTWPSO(instance,options,[]); %#ok<ASGLU>
                first=stats.firstFeasibleFE; if isinf(first), first=NaN; end
                if best.Detail.isFeasible, gap=(best.Detail.tourCost-refCost)/refCost; else, gap=NaN; end
                rows{end+1,1}={manifest.name(i),instance.nCustomers,methods(m), ...
                    seed,factor,budget,history.FE(end),best.Detail.totalLate, ...
                    best.Detail.isFeasible,first,best.Detail.tourCost,gap, ...
                    stats.feedbackUpdates}; %#ok<AGROW>
            end
        end
    end
    fprintf('%s completed.\n',manifest.name(i));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'instance','nCustomers','method','seed','budget_factor','budget_fe', ...
    'actual_fe','total_late','is_feasible','first_feasible_fe','tour_cost', ...
    'gap_to_bks','feedback_updates'});
results.summary=summary; results.methods=methods; results.factors=factors;
writetable(summary,fullfile(root,'results','feedback_ablation_tsptw_summary.csv'));
save(fullfile(root,'results','feedback_ablation_tsptw_result.mat'),'results');
fprintf('Feedback ablation completed: %d rows.\n',height(summary));
end
