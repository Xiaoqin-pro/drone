function results = RunBoundaryVNDDecision100n
%RUNBOUNDARYVNDDECISION100N 最终候选Late/Global/2opt/VND机制决策实验
root=fileparts(fileparts(mfilename('fullpath'))); setup;
rawDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
reference=ReadTSPTWReferenceSolutions(fullfile(root,'data','tsp_tw_raw','best_known','SolomonPotvinBengio-best-known-traveltime.txt'));
selected=["rc_201.1.txt","rc_201.2.txt","rc_201.3.txt","rc_202.1.txt", ...
    "rc_202.3.txt","rc_204.2.txt","rc_207.3.txt","rc_204.1.txt","rc_208.3.txt"];
methods=["LateOnly","LateToGlobal","LateTo2Opt","LateToVND"]; seeds=1:10; rows=cell(0,1);
for i=1:numel(selected)
    fileName=selected(i); instance=ReadTSPTWInstance(fullfile(rawDir,fileName));
    refIndex=find(strcmpi(string({reference.file}),fileName),1); if isempty(refIndex), refCost=NaN; else, refCost=reference(refIndex).cost; end
    n=instance.nCustomers; sourceFE=max(1,n-1); vndFE=3*sourceFE; budget=100*n;
    for seedIndex=1:numel(seeds)
        seed=997000+100*i+seedIndex;
        for m=1:numel(methods)
            rng(seed,'twister'); options=struct('nPop',20,'maxFE',budget,'latePenalty',1000, ...
                'localSearchMode','late','localSearchFE',sourceFE,'restorationFE',sourceFE, ...
                'intensificationFE',vndFE,'silent',true);
            switch methods(m)
                case "LateToGlobal", options.localSearchMode='state-switch-global';
                case "LateTo2Opt", options.localSearchMode='state-switch-2opt';
                case "LateToVND", options.localSearchMode='state-switch'; options.localSearchFE=vndFE;
            end
            [best,~,stats]=RandomKeyTSPTWPSO(instance,options,[]); %#ok<ASGLU>
            feasible=best.Detail.isFeasible; first=stats.firstFeasibleFE; if isinf(first), first=NaN; end
            phase=stats.phaseSwitchFE; if isinf(phase), phase=NaN; end
            if feasible && isfinite(refCost), gap=(best.Detail.tourCost-refCost)/refCost; else, gap=NaN; end
            firstCost=NaN; if ~isempty(stats.firstFeasibleDetail), firstCost=stats.firstFeasibleDetail.tourCost; end
            rows{end+1,1}={fileName,n,methods(m),seed,budget,feasible,first,phase, ...
                stats.phaseSwitchCost,stats.phaseGain,stats.phaseGainPerFE,best.Detail.totalLate, ...
                best.Detail.tourCost,firstCost,gap,stats.restorationRelocateFE, ...
                stats.restorationRelocateAccept,stats.vndTwoOptAccept,stats.vndSwapAccept, ...
                stats.vndRelocateAccept}; %#ok<AGROW>
        end
    end
    fprintf('%s completed.\n',fileName);
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','nCustomers','method','seed','budget_fe','is_feasible','first_feasible_fe', ...
    'phase_switch_fe','phase_switch_cost','phase_gain','phase_gain_per_fe','total_late', ...
    'final_cost','first_feasible_cost','gap_to_bks','restoration_relocate_fe', ...
    'restoration_relocate_accept','vnd_2opt_accept','vnd_swap_accept','vnd_relocate_accept'});
results.summary=summary; results.methods=methods; results.selected=selected;
writetable(summary,fullfile(root,'results','boundary_vnd_decision_100n_summary.csv'));
save(fullfile(root,'results','boundary_vnd_decision_100n_result.mat'),'results');
fprintf('Boundary VND decision completed: %d rows.\n',height(summary));
end
