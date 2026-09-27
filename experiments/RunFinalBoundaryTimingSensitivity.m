function results = RunFinalBoundaryTimingSensitivity(nSeeds)
%RUNFINALBOUNDARYTIMINGSENSITIVITY 最终版边界切换时机敏感性实验
%   比较 FirstFeasible、Scheduled50、Scheduled75，保持同一paired seed。
if nargin<1 || isempty(nSeeds), nSeeds=30; end
root=fileparts(fileparts(mfilename('fullpath'))); setup;
rawDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
files=["rc_201.1.txt","rc_201.2.txt","rc_201.3.txt","rc_202.1.txt", ...
    "rc_202.3.txt","rc_204.2.txt","rc_207.3.txt","rc_204.1.txt","rc_208.3.txt"];
methods=["FirstFeasible","Scheduled50","Scheduled75"]; seeds=1:nSeeds; rows=cell(0,1);
for i=1:numel(files)
    instance=ReadTSPTWInstance(fullfile(rawDir,files(i)));
    n=instance.nCustomers; q=max(1,n-1); budget=100*n;
    for s=1:numel(seeds)
        seed=1401000+100*i+s;
        for m=1:numel(methods)
            rng(seed,'twister');
            options=struct('nPop',20,'maxFE',budget,'latePenalty',1000, ...
                'localSearchMode','state-switch','localSearchFE',3*q, ...
                'restorationFE',q,'intensificationFE',3*q,'silent',true);
            requestedTransition=NaN;
            if methods(m)=="Scheduled50"
                requestedTransition=round(0.50*budget);
                options.localSearchMode='scheduled-vnd';
                options.transitionFE=requestedTransition;
            elseif methods(m)=="Scheduled75"
                requestedTransition=round(0.75*budget);
                options.localSearchMode='scheduled-vnd';
                options.transitionFE=requestedTransition;
            end
            [best,~,stats]=RandomKeyTSPTWPSO(instance,options,[]); %#ok<ASGLU>
            first=stats.firstFeasibleFE; if isinf(first), first=NaN; end
            phase=stats.phaseSwitchFE; if isinf(phase), phase=NaN; end
            rows{end+1,1}={files(i),n,methods(m),s,budget,requestedTransition, ...
                best.Detail.isFeasible,first,phase,best.Detail.totalLate, ...
                best.Detail.tourCost,stats.phaseGain,stats.functionEvaluations}; %#ok<AGROW>
        end
    end
    fprintf('%s completed (%d seeds).\n',files(i),nSeeds);
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','nCustomers','method','seed','budget_fe','requested_transition_fe', ...
    'is_feasible','first_feasible_fe','phase_switch_fe','total_late','final_cost', ...
    'phase_gain','function_evaluations'});
results.summary=summary; results.methods=methods; results.seeds=seeds;
outBase=fullfile(root,'results',sprintf('final_boundary_timing_%dseeds',nSeeds));
writetable(summary,[outBase '_summary.csv']);
save([outBase '_result.mat'],'results','-v7.3');
fprintf('Final boundary timing sensitivity completed: %d rows.\n',height(summary));
end
