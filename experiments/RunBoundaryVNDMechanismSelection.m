function results = RunBoundaryVNDMechanismSelection
%RUNBOUNDARYVNDMECHANISMSELECTION 在筛选后的SPB实例上比较Late/Global/2opt/VND

root=fileparts(fileparts(mfilename('fullpath'))); setup;
rawDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
refFile=fullfile(root,'data','tsp_tw_raw','best_known','SolomonPotvinBengio-best-known-traveltime.txt');
reference=ReadTSPTWReferenceSolutions(refFile);
selected=["rc_201.1.txt","rc_201.2.txt","rc_201.3.txt","rc_202.1.txt", ...
    "rc_202.3.txt","rc_204.2.txt","rc_207.3.txt","rc_204.1.txt","rc_208.3.txt"];
methods=["LateOnly","LateToGlobal","LateTo2Opt","LateToVND"];
seeds=1:10; factors=[25 50 100]; rows=cell(0,1);
for i=1:numel(selected)
    fileName=selected(i); instance=ReadTSPTWInstance(fullfile(rawDir,fileName));
    refIndex=find(strcmpi(string({reference.file}),fileName),1);
    if isempty(refIndex), refCost=NaN; else, refCost=reference(refIndex).cost; end
    n=instance.nCustomers; sourceFE=max(1,n-1); vndFE=3*sourceFE;
    for seedIndex=1:numel(seeds)
        seed=996000+100*i+seedIndex;
        for factor=factors
            budget=factor*n;
            for m=1:numel(methods)
                rng(seed,'twister');
                options=struct('nPop',20,'maxFE',budget,'latePenalty',1000, ...
                    'localSearchMode','late','localSearchFE',sourceFE, ...
                    'restorationFE',sourceFE,'intensificationFE',vndFE,'silent',true);
                switch methods(m)
                    case "LateToGlobal"
                        options.localSearchMode='state-switch-global';
                    case "LateTo2Opt"
                        options.localSearchMode='state-switch-2opt';
                    case "LateToVND"
                        options.localSearchMode='state-switch'; options.localSearchFE=vndFE;
                end
                [best,history,stats]=RandomKeyTSPTWPSO(instance,options,[]); %#ok<ASGLU>
                first=stats.firstFeasibleFE; if isinf(first), first=NaN; end
                phase=stats.phaseSwitchFE; if isinf(phase), phase=NaN; end
                feasible=best.Detail.isFeasible;
                if feasible && isfinite(refCost), gap=(best.Detail.tourCost-refCost)/refCost; else, gap=NaN; end
                firstCost=NaN; postGain=NaN; relPostGain=NaN;
                if ~isempty(stats.firstFeasibleDetail)
                    firstCost=stats.firstFeasibleDetail.tourCost;
                    if feasible
                        postGain=firstCost-best.Detail.tourCost;
                        relPostGain=postGain/max(abs(firstCost),eps);
                    end
                end
                vnd2=0; vnds=0; vndr=0;
                if isfield(stats,'localSearchTwoOptFE'), vnd2=stats.localSearchTwoOptFE; end
                if isfield(stats,'localSearchSwapFE'), vnds=stats.localSearchSwapFE; end
                if isfield(stats,'localSearchRelocateFE'), vndr=stats.localSearchRelocateFE; end
                rows{end+1,1}={fileName,n,methods(m),seed,factor,budget, ...
                    history.FE(end),feasible,stats.firstFeasibleFE,phase, ...
                    best.Detail.totalLate,best.Detail.tourCost,firstCost,postGain, ...
                    relPostGain,gap,vnd2,vnds,vndr}; %#ok<AGROW>
            end
        end
    end
    fprintf('%s completed.\n',fileName);
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','nCustomers','method','seed','budget_factor','budget_fe','actual_fe', ...
    'is_feasible','first_feasible_fe','phase_switch_fe','total_late','final_cost', ...
    'first_feasible_cost','post_feasible_gain','relative_post_gain','gap_to_bks', ...
    'vnd_2opt_fe','vnd_swap_fe','vnd_relocate_fe'});
results.summary=summary; results.selected=selected; results.methods=methods; results.factors=factors;
writetable(summary,fullfile(root,'results','boundary_vnd_mechanism_selection_summary.csv'));
save(fullfile(root,'results','boundary_vnd_mechanism_selection_result.mat'),'results');
fprintf('Boundary VND mechanism selection completed: %d rows.\n',height(summary));
end
