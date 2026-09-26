function results = RunTSPTWFeasibilityAudit
%RUNTSPTWFEASIBILITYAUDIT 检验可行性恢复Relocate是否更快进入可行域

root=fileparts(fileparts(mfilename('fullpath'))); setup;
benchmarkDir=fullfile(root,'data','tsp_tw_benchmark');
manifest=readtable(fullfile(benchmarkDir,'manifest.csv'),'Delimiter',',','TextType','string');
selected=1:height(manifest); methods=["RandomKeyPSO","PSO_GlobalRelocate","PSO_FeasibilityRelocate"];
seeds=1:10; rows=cell(0,1);
for i=selected
    instance=ReadTSPTWInstance(fullfile(benchmarkDir,manifest.file(i)));
    maxFE=50*instance.nCustomers; checkpoints=[25*instance.nCustomers,maxFE];
    for seedIndex=1:numel(seeds)
        seed=930000+100*i+seeds(seedIndex);
        for m=1:numel(methods)
            rng(seed,'twister'); options=struct('nPop',20,'maxFE',maxFE, ...
                'latePenalty',1000,'localSearchFE',0,'localSearchMode','none');
            if methods(m)=="PSO_GlobalRelocate"
                options.localSearchMode='relocate'; options.localSearchFE=25;
            elseif methods(m)=="PSO_FeasibilityRelocate"
                options.localSearchMode='feasibility'; options.localSearchFE=25;
            end
            [best,history,stats]=RandomKeyTSPTWPSO(instance,options,[]); %#ok<ASGLU>
            for cp=checkpoints
                idx=find(history.FE<=cp,1,'last'); if isempty(idx), continue; end
                firstFeasible=stats.firstFeasibleFE; if isinf(firstFeasible), firstFeasible=NaN; end
                rows{end+1,1}={manifest.name(i),instance.nCustomers,methods(m), ...
                    seed,cp,history.FE(idx),history.Late(idx),history.IsFeasible(idx), ...
                    firstFeasible,history.TourCost(idx)}; %#ok<AGROW>
            end
        end
    end
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'instance','nCustomers','method','seed','checkpoint_fe','actual_fe', ...
    'total_late','is_feasible','first_feasible_fe','tour_cost'});
results.summary=summary; results.methods=methods;
writetable(summary,fullfile(root,'results','tsptw_feasibility_audit_summary.csv'));
save(fullfile(root,'results','tsptw_feasibility_audit_result.mat'),'results');
fprintf('TSPTW feasibility audit completed: %d rows.\n',height(summary));
end
