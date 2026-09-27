function results = RunAllSPBMechanismScreen
%RUNALLSPBMECHANISMSCREEN 在全部30个SPB实例上进行Late/2opt/VND筛选
%   这是正式大实验前的全量筛选，不包含外部phase-transition实例。

root=fileparts(fileparts(mfilename('fullpath'))); setup;
rawDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
reference=ReadTSPTWReferenceSolutions(fullfile(root,'data','tsp_tw_raw','best_known', ...
    'SolomonPotvinBengio-best-known-traveltime.txt'));
files=dir(fullfile(rawDir,'*.txt'));
methods=["LateOnly","LateTo2Opt","LateToVND"];
seeds=1:10; rows=cell(0,1);
for i=1:numel(files)
    fileName=string(files(i).name);
    instance=ReadTSPTWInstance(fullfile(files(i).folder,files(i).name));
    refIndex=find(strcmpi(string({reference.file}),fileName),1);
    if isempty(refIndex), refCost=NaN; else, refCost=reference(refIndex).cost; end
    n=instance.nCustomers; q=max(1,n-1); budget=100*n;
    for seedIndex=1:numel(seeds)
        seed=1001000+100*i+seedIndex;
        for m=1:numel(methods)
            rng(seed,'twister');
            options=struct('nPop',20,'maxFE',budget,'latePenalty',1000, ...
                'localSearchMode','late','localSearchFE',q, ...
                'restorationFE',q,'intensificationFE',3*q,'silent',true);
            if methods(m)=="LateTo2Opt"
                options.localSearchMode='state-switch-2opt'; options.localSearchFE=q;
            elseif methods(m)=="LateToVND"
                options.localSearchMode='state-switch'; options.localSearchFE=3*q;
            end
            [best,~,stats]=RandomKeyTSPTWPSO(instance,options,[]); %#ok<ASGLU>
            feasible=best.Detail.isFeasible;
            first=stats.firstFeasibleFE; if isinf(first), first=NaN; end
            phase=stats.phaseSwitchFE; if isinf(phase), phase=NaN; end
            if feasible && isfinite(refCost), gap=(best.Detail.tourCost-refCost)/refCost; else, gap=NaN; end
            firstCost=NaN; if ~isempty(stats.firstFeasibleDetail), firstCost=stats.firstFeasibleDetail.tourCost; end
            rows{end+1,1}={fileName,n,methods(m),seed,budget,feasible,first,phase, ...
                stats.phaseGain,stats.phaseGainPerFE,best.Detail.totalLate,best.Detail.tourCost, ...
                firstCost,gap,stats.restorationRelocateFE,stats.restorationRelocateAccept, ...
                stats.vndTwoOptAccept,stats.vndSwapAccept,stats.vndRelocateAccept}; %#ok<AGROW>
        end
    end
    fprintf('%s completed.\n',fileName);
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','nCustomers','method','seed','budget_fe','is_feasible','first_feasible_fe', ...
    'phase_switch_fe','phase_gain','phase_gain_per_fe','total_late','final_cost', ...
    'first_feasible_cost','gap_to_bks','restoration_relocate_fe', ...
    'restoration_relocate_accept','vnd_2opt_accept','vnd_swap_accept','vnd_relocate_accept'});
results.summary=summary; results.methods=methods; results.seeds=seeds;
writetable(summary,fullfile(root,'results','all_spb_mechanism_screen_summary.csv'));
save(fullfile(root,'results','all_spb_mechanism_screen_result.mat'),'results','-v7.3');
fprintf('All SPB mechanism screen completed: %d rows.\n',height(summary));
end
