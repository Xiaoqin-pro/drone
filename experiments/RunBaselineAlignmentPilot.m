function results = RunBaselineAlignmentPilot
%RUNBASELINEALIGNMENTPILOT 对齐PSO、LateOnly、FB-CMPSO和Clean GVNS
root=fileparts(fileparts(mfilename('fullpath'))); setup;
rawDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
reference=ReadTSPTWReferenceSolutions(fullfile(root,'data','tsp_tw_raw','best_known', ...
    'SolomonPotvinBengio-best-known-traveltime.txt'));
files=["rc_201.1.txt","rc_201.3.txt","rc_202.1.txt","rc_204.2.txt", ...
    "rc_207.3.txt","rc_208.3.txt"];
methods=["RandomKeyPSO","LateOnly","FB-CMPSO","CleanGVNS"]; seeds=1:5; rows=cell(0,1);
for i=1:numel(files)
    instance=ReadTSPTWInstance(fullfile(rawDir,files(i))); n=instance.nCustomers; q=max(1,n-1); budget=100*n;
    refIndex=find(strcmpi(string({reference.file}),files(i)),1); if isempty(refIndex), refCost=NaN; else, refCost=reference(refIndex).cost; end
    for seedIndex=1:numel(seeds)
        seed=1200000+100*i+seedIndex;
        for m=1:numel(methods)
            rng(seed,'twister');
            if methods(m)=="CleanGVNS"
                options=struct('maxFE',budget,'maxNeighborhood',3,'localSearchFE',q, ...
                    'latePenalty',1000,'silent',true);
                tic; [~,detail,stats,~]=GVNSTSPTW(instance,options); responseTime=toc; %#ok<ASGLU>
                phaseSwitch=NaN; phaseGain=NaN; finalCost=detail.tourCost; late=detail.totalLate; feasible=detail.isFeasible;
            else
                options=struct('nPop',20,'maxFE',budget,'latePenalty',1000, ...
                    'localSearchMode','none','localSearchFE',0,'restorationFE',q, ...
                    'intensificationFE',3*q,'silent',true);
                if methods(m)=="LateOnly"
                    options.localSearchMode='late'; options.localSearchFE=q;
                elseif methods(m)=="FB-CMPSO"
                    options.localSearchMode='state-switch'; options.localSearchFE=3*q;
                end
                tic; [best,~,stats]=RandomKeyTSPTWPSO(instance,options,[]); responseTime=toc;
                detail=best.Detail; finalCost=detail.tourCost; late=detail.totalLate; feasible=detail.isFeasible;
                phaseSwitch=stats.phaseSwitchFE; if isinf(phaseSwitch), phaseSwitch=NaN; end
                phaseGain=stats.phaseGain;
            end
            first=stats.firstFeasibleFE; if isinf(first), first=NaN; end
            if feasible && isfinite(refCost), gap=(finalCost-refCost)/refCost; else, gap=NaN; end
            rows{end+1,1}={files(i),n,methods(m),seed,budget,stats.functionEvaluations, ...
                feasible,first,phaseSwitch,phaseGain,late,finalCost,gap,responseTime}; %#ok<AGROW>
        end
    end
    fprintf('%s completed.\n',files(i));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','nCustomers','method','seed','budget_fe','actual_fe','is_feasible', ...
    'first_feasible_fe','phase_switch_fe','phase_gain','total_late','final_cost', ...
    'gap_to_bks','response_time'});
results.summary=summary; results.methods=methods; results.files=files; results.seeds=seeds;
writetable(summary,fullfile(root,'results','baseline_alignment_pilot_summary.csv'));
save(fullfile(root,'results','baseline_alignment_pilot_result.mat'),'results');
fprintf('Baseline alignment pilot completed: %d rows.\n',height(summary));
end


