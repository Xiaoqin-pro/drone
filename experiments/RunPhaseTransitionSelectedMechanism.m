function results = RunPhaseTransitionSelectedMechanism
%RUNPHASETRANSITIONSELECTEDMECHANISM 在筛选出的boundary-active外部实例上验证三种方法
root=fileparts(fileparts(mfilename('fullpath'))); setup;
base=fullfile(root,'data','phase_transition_extracted');
selected=readtable(fullfile(root,'results','phase_transition_selected_boundary.csv'),'TextType','string');
methods=["LateOnly","LateTo2Opt","LateToVND"]; seeds=1:10; rows=cell(0,1);
for i=1:height(selected)
    nPoints=selected.benchmark_nodes(i); beta=selected.beta(i); realization=selected.realization(i);
    filePath=fullfile(base,sprintf('n%d',nPoints),sprintf('n=%d',nPoints), ...
        sprintf('ins_n_%d_alph_1.00_beta_%.2f_%d.txt',nPoints,beta,realization));
    if ~isfile(filePath), warning('missing %s',filePath); continue; end
    instance=ReadTSPTWInstance(filePath); n=instance.nCustomers; q=max(1,n-1); budget=500*n;
    for seedIndex=1:numel(seeds)
        seed=1030000+100*i+seedIndex;
        for m=1:numel(methods)
            rng(seed,'twister'); options=struct('nPop',20,'maxFE',budget, ...
                'latePenalty',1000,'localSearchMode','late','localSearchFE',q, ...
                'restorationFE',q,'intensificationFE',3*q,'silent',true);
            if methods(m)=="LateTo2Opt"
                options.localSearchMode='state-switch-2opt';
            elseif methods(m)=="LateToVND"
                options.localSearchMode='state-switch'; options.localSearchFE=3*q;
            end
            [best,~,stats]=RandomKeyTSPTWPSO(instance,options,[]); %#ok<ASGLU>
            first=stats.firstFeasibleFE; if isinf(first), first=NaN; end
            phase=stats.phaseSwitchFE; if isinf(phase), phase=NaN; end
            rows{end+1,1}={nPoints,beta,realization,filePath,methods(m),seed,budget, ...
                best.Detail.isFeasible,first,phase,best.Detail.totalLate,best.Detail.tourCost, ...
                stats.phaseGain,stats.phaseGainPerFE}; %#ok<AGROW>
        end
    end
    fprintf('selected candidate %d/%d completed.\n',i,height(selected));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'benchmark_nodes','beta','realization','file','method','seed','budget_fe', ...
    'is_feasible','first_feasible_fe','phase_switch_fe','total_late','final_cost', ...
    'phase_gain','phase_gain_per_fe'});
results.summary=summary; results.methods=methods; results.seeds=seeds;
writetable(summary,fullfile(root,'results','phase_transition_selected_mechanism_summary.csv'));
save(fullfile(root,'results','phase_transition_selected_mechanism_result.mat'),'results');
fprintf('Selected phase-transition mechanism run completed: %d rows.\n',height(summary));
end
