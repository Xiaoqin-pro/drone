function results = RunCoreMechanismFreezeTSPTW
%RUNCOREMECHANISMFREEZETSPTW PSO/Global/Late/Propagation/StateSwitch机制冻结
%   每个FE预算独立启动，但实例和算法seed严格配对；source邻域预算=nCustomers。

root=fileparts(fileparts(mfilename('fullpath'))); setup;
benchmarkDir=fullfile(root,'data','tsp_tw_benchmark');
manifest=readtable(fullfile(benchmarkDir,'manifest.csv'),'Delimiter',',','TextType','string');
reference=ReadTSPTWReferenceSolutions(fullfile(root,'data','tsp_tw_raw','best_known', ...
    'SolomonPotvinBengio-best-known-traveltime.txt'));
methods=["PSO","LateOnly","LateToGlobal","LateTo2Opt","StateSwitch"];
seeds=1:10; budgetsFactor=[25 50 100]; rows=cell(0,1);
for i=1:height(manifest)
    instance=ReadTSPTWInstance(fullfile(benchmarkDir,manifest.file(i)));
    refIndex=find(strcmpi(string({reference.file}),manifest.file(i)),1);
    refCost=reference(refIndex).cost;
    for seedIndex=1:numel(seeds)
        seed=940000+100*i+seeds(seedIndex);
        for budgetFactor=budgetsFactor
            budget=budgetFactor*instance.nCustomers;
            for m=1:numel(methods)
                rng(seed,'twister');
                options=struct('nPop',20,'maxFE',budget,'latePenalty',1000, ...
                    'localSearchFE',0,'localSearchMode','none','silent',true);
                if methods(m)=="LateOnly"
                    options.localSearchMode='late'; options.localSearchFE=max(1,instance.nCustomers-1);
                elseif methods(m)=="LateToGlobal"
                    options.localSearchMode='state-switch-global'; options.localSearchFE=max(1,instance.nCustomers-1);
                elseif methods(m)=="LateTo2Opt"
                    options.localSearchMode='state-switch-2opt'; options.localSearchFE=max(1,instance.nCustomers-1);
                elseif methods(m)=="StateSwitch"
                    options.localSearchMode='state-switch'; options.localSearchFE=3*max(1,instance.nCustomers-1);
                end
                [best,history,stats]=RandomKeyTSPTWPSO(instance,options,[]); %#ok<ASGLU>
                idx=numel(history.FE);
                first=stats.firstFeasibleFE; if isinf(first), first=NaN; end
                if best.Detail.isFeasible, gap=(best.Detail.tourCost-refCost)/refCost; else, gap=NaN; end
                mode=""; if isfield(history,'Mode'), mode=history.Mode(idx); end
                if isfield(stats,'phaseSwitchFE'), phaseSwitchFE=stats.phaseSwitchFE; else, phaseSwitchFE=NaN; end
                if isfield(stats,'localSearchTwoOptFE'), vnd2optFE=stats.localSearchTwoOptFE; else, vnd2optFE=0; end
                if isfield(stats,'localSearchSwapFE'), vndSwapFE=stats.localSearchSwapFE; else, vndSwapFE=0; end
                if isfield(stats,'localSearchRelocateFE'), vndRelocateFE=stats.localSearchRelocateFE; else, vndRelocateFE=0; end
                rows{end+1,1}={manifest.name(i),instance.nCustomers,methods(m), ...
                    seed,budgetFactor,budget,history.FE(idx),history.Late(idx), ...
                    history.IsFeasible(idx),first,phaseSwitchFE,best.Detail.tourCost,gap, ...
                    mode,vnd2optFE,vndSwapFE,vndRelocateFE}; %#ok<AGROW>
            end
        end
    end
    fprintf('%s completed.\n',manifest.name(i));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'instance','nCustomers','method','seed','budget_factor','budget_fe', ...
    'actual_fe','total_late','is_feasible','first_feasible_fe','phase_switch_fe', ...
    'tour_cost','gap_to_bks','mode','vnd_2opt_fe','vnd_swap_fe','vnd_relocate_fe'});
results.summary=summary; results.methods=methods; results.budgetsFactor=budgetsFactor;
writetable(summary,fullfile(root,'results','core_mechanism_freeze_tsptw_summary.csv'));
save(fullfile(root,'results','core_mechanism_freeze_tsptw_result.mat'),'results');
fprintf('Core mechanism freeze completed: %d rows.\n',height(summary));
end
