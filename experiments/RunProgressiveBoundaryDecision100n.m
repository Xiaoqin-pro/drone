function results = RunProgressiveBoundaryDecision100n
%RUNPROGRESSIVEBOUNDARYDECISION100N 比较LateTo2Opt/VND/Progressive
root=fileparts(fileparts(mfilename('fullpath'))); setup;
rawDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
reference=ReadTSPTWReferenceSolutions(fullfile(root,'data','tsp_tw_raw','best_known','SolomonPotvinBengio-best-known-traveltime.txt'));
selected=["rc_201.1.txt","rc_201.2.txt","rc_201.3.txt","rc_202.1.txt", ...
    "rc_202.3.txt","rc_204.2.txt","rc_207.3.txt","rc_204.1.txt","rc_208.3.txt"];
methods=["LateTo2Opt","LateToVND","LateToProgressive"]; seeds=1:10; rows=cell(0,1);
for i=1:numel(selected)
    fileName=selected(i); instance=ReadTSPTWInstance(fullfile(rawDir,fileName));
    refIndex=find(strcmpi(string({reference.file}),fileName),1); if isempty(refIndex), refCost=NaN; else, refCost=reference(refIndex).cost; end
    n=instance.nCustomers; q=max(1,n-1); budget=100*n;
    for seedIndex=1:numel(seeds)
        seed=999000+100*i+seedIndex;
        for m=1:numel(methods)
            rng(seed,'twister'); options=struct('nPop',20,'maxFE',budget,'latePenalty',1000, ...
                'localSearchFE',3*q,'restorationFE',q,'intensificationFE',3*q,'silent',true);
            if methods(m)=="LateTo2Opt"
                options.localSearchMode='state-switch-2opt'; options.localSearchFE=q;
            elseif methods(m)=="LateToVND"
                options.localSearchMode='state-switch';
            else
                options.localSearchMode='state-switch-progressive';
            end
            [best,~,stats]=RandomKeyTSPTWPSO(instance,options,[]); %#ok<ASGLU>
            feasible=best.Detail.isFeasible; first=stats.firstFeasibleFE; if isinf(first), first=NaN; end
            phase=stats.phaseSwitchFE; if isinf(phase), phase=NaN; end
            if feasible && isfinite(refCost), gap=(best.Detail.tourCost-refCost)/refCost; else, gap=NaN; end
            rows{end+1,1}={fileName,methods(m),seed,feasible,first,phase, ...
                stats.phaseGain,stats.phaseGainPerFE,best.Detail.totalLate,best.Detail.tourCost,gap, ...
                stats.vndTwoOptAccept,stats.vndSwapAccept,stats.vndRelocateAccept,stats.localSearchFE}; %#ok<AGROW>
        end
    end
    fprintf('%s completed.\n',fileName);
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','method','seed','is_feasible','first_feasible_fe','phase_switch_fe', ...
    'phase_gain','phase_gain_per_fe','total_late','final_cost','gap_to_bks', ...
    'vnd_2opt_accept','vnd_swap_accept','vnd_relocate_accept','local_search_fe'});
results.summary=summary; results.methods=methods;
writetable(summary,fullfile(root,'results','progressive_boundary_decision_100n.csv'));
save(fullfile(root,'results','progressive_boundary_decision_100n.mat'),'results');
fprintf('Progressive boundary decision completed: %d rows.\n',height(summary));
end
