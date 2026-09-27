function results = RunPhaseTransitionPilot
%RUNPHASETRANSITIONPILOT 小规模独立phase-transition基准试验
%   使用公开phase-transition实例的n=21/31、alpha=1.00和多个beta切片；
%   先比较LateOnly/LateTo2Opt/LateToVND，不引入BKS Gap。

root=fileparts(fileparts(mfilename('fullpath'))); setup;
base=fullfile(root,'data','phase_transition_extracted');
patterns=["n21\\n=21\\ins_n_21_alph_1.00_beta_0.30_1.txt", ...
    "n21\\n=21\\ins_n_21_alph_1.00_beta_0.50_1.txt", ...
    "n21\\n=21\\ins_n_21_alph_1.00_beta_0.70_1.txt", ...
    "n21\\n=21\\ins_n_21_alph_1.00_beta_0.90_1.txt", ...
    "n31\\n=31\\ins_n_31_alph_1.00_beta_0.30_1.txt", ...
    "n31\\n=31\\ins_n_31_alph_1.00_beta_0.50_1.txt", ...
    "n31\\n=31\\ins_n_31_alph_1.00_beta_0.70_1.txt", ...
    "n31\\n=31\\ins_n_31_alph_1.00_beta_0.90_1.txt"];
methods=["LateOnly","LateTo2Opt","LateToVND"]; seeds=1:5; rows=cell(0,1);
for i=1:numel(patterns)
    filePath=fullfile(base,patterns(i));
    if ~isfile(filePath), warning('missing: %s',filePath); continue; end
    instance=ReadTSPTWInstance(filePath); n=instance.nCustomers; q=max(1,n-1); budget=100*n;
    for seedIndex=1:numel(seeds)
        seed=1010000+100*i+seedIndex;
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
            rows{end+1,1}={patterns(i),n,methods(m),seed,budget,best.Detail.isFeasible, ...
                first,phase,best.Detail.totalLate,best.Detail.tourCost,stats.phaseGain, ...
                stats.phaseGainPerFE}; %#ok<AGROW>
        end
    end
    fprintf('%s completed.\n',patterns(i));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','nCustomers','method','seed','budget_fe','is_feasible', ...
    'first_feasible_fe','phase_switch_fe','total_late','final_cost','phase_gain', ...
    'phase_gain_per_fe'});
results.summary=summary; results.methods=methods; results.seeds=seeds;
writetable(summary,fullfile(root,'results','phase_transition_pilot_summary.csv'));
save(fullfile(root,'results','phase_transition_pilot_result.mat'),'results');
fprintf('Phase-transition pilot completed: %d rows.\n',height(summary));
end
