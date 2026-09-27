function results = RunFinalSPBMechanismScreen(nSeeds)
%RUNFINALSPBMECHANISMSCREEN 最终版30个SPB机制对照实验
%   方法：RKPSO、LateOnly、LateTo2Opt、AlwaysVND、FB-CMPSO。
%   各方法使用相同的100*n FE预算和paired seeds。
if nargin<1 || isempty(nSeeds), nSeeds=30; end
root=fileparts(fileparts(mfilename('fullpath'))); setup;
rawDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
reference=ReadTSPTWReferenceSolutions(fullfile(root,'data','tsp_tw_raw','best_known', ...
    'SolomonPotvinBengio-best-known-traveltime.txt'));
files=dir(fullfile(rawDir,'*.txt'));
methods=["RKPSO","LateOnly","LateTo2Opt","AlwaysVND","FB-CMPSO"];
seeds=1:nSeeds; rows=cell(0,1);
for i=1:numel(files)
    fileName=string(files(i).name);
    instance=ReadTSPTWInstance(fullfile(files(i).folder,files(i).name));
    refIndex=find(strcmpi(string({reference.file}),fileName),1);
    if isempty(refIndex), refCost=NaN; else, refCost=reference(refIndex).cost; end
    n=instance.nCustomers; q=max(1,n-1); budget=100*n;
    for seedIndex=1:numel(seeds)
        seed=1301000+100*i+seedIndex;
        for m=1:numel(methods)
            rng(seed,'twister');
            options=struct('nPop',20,'maxFE',budget,'latePenalty',1000, ...
                'localSearchMode','none','localSearchFE',0, ...
                'restorationFE',q,'intensificationFE',3*q,'silent',true);
            switch methods(m)
                case "LateOnly"
                    options.localSearchMode='late'; options.localSearchFE=q;
                case "LateTo2Opt"
                    options.localSearchMode='state-switch-2opt'; options.localSearchFE=q;
                case "AlwaysVND"
                    options.localSearchMode='budgeted-vnd'; options.localSearchFE=3*q;
                case "FB-CMPSO"
                    options.localSearchMode='state-switch'; options.localSearchFE=3*q;
            end
            [best,~,stats]=RandomKeyTSPTWPSO(instance,options,[]); %#ok<ASGLU>
            first=stats.firstFeasibleFE; if isinf(first), first=NaN; end
            phase=stats.phaseSwitchFE; if isinf(phase), phase=NaN; end
            if best.Detail.isFeasible && isfinite(refCost)
                gap=(best.Detail.tourCost-refCost)/refCost;
            else
                gap=NaN;
            end
            firstCost=NaN;
            if ~isempty(stats.firstFeasibleDetail), firstCost=stats.firstFeasibleDetail.tourCost; end
            rows{end+1,1}={fileName,n,methods(m),seed,budget,best.Detail.isFeasible, ...
                first,phase,stats.phaseGain,stats.phaseGainPerFE,best.Detail.totalLate, ...
                best.Detail.tourCost,firstCost,gap,stats.restorationRelocateFE, ...
                stats.restorationRelocateAccept,stats.vndTwoOptAccept, ...
                stats.vndSwapAccept,stats.vndRelocateAccept,stats.functionEvaluations}; %#ok<AGROW>
        end
    end
    fprintf('%s completed (%d seeds).\n',fileName,nSeeds);
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','nCustomers','method','seed','budget_fe','is_feasible','first_feasible_fe', ...
    'phase_switch_fe','phase_gain','phase_gain_per_fe','total_late','final_cost', ...
    'first_feasible_cost','gap_to_bks','restoration_relocate_fe', ...
    'restoration_relocate_accept','vnd_2opt_accept','vnd_swap_accept', ...
    'vnd_relocate_accept','function_evaluations'});
results.summary=summary; results.methods=methods; results.seeds=seeds;
outBase=fullfile(root,'results',sprintf('final_spb_mechanism_%dseeds',nSeeds));
writetable(summary,[outBase '_summary.csv']);
save([outBase '_result.mat'],'results','-v7.3');
fprintf('Final SPB mechanism screen completed: %d rows.\n',height(summary));
end
