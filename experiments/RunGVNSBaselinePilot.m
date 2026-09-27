function results = RunGVNSBaselinePilot
%RUNG VNSBASELINEPILOT 在代表性SPB实例上验证GVNS基线接口
root=fileparts(fileparts(mfilename('fullpath'))); setup;
rawDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
files=["rc_201.1.txt","rc_201.3.txt","rc_202.1.txt","rc_204.2.txt", ...
    "rc_207.3.txt","rc_208.3.txt"];
seeds=1:5; rows=cell(0,1);
for i=1:numel(files)
    instance=ReadTSPTWInstance(fullfile(rawDir,files(i))); budget=100*instance.nCustomers;
    for s=1:numel(seeds)
        rng(1100000+100*i+s,'twister');
        options=struct('maxFE',budget,'maxNeighborhood',3,'localSearchFE', ...
            max(1,instance.nCustomers-1),'latePenalty',1000,'silent',true);
        [bestRoute,bestDetail,stats,~]=GVNSTSPTW(instance,options); %#ok<ASGLU>
        rows{end+1,1}={files(i),instance.nCustomers,s,budget,bestDetail.isFeasible, ...
            stats.firstFeasibleFE,bestDetail.totalLate,bestDetail.tourCost,stats.functionEvaluations}; %#ok<AGROW>
    end
    fprintf('%s completed.\n',files(i));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','nCustomers','seed','budget_fe','is_feasible','first_feasible_fe', ...
    'total_late','tour_cost','actual_fe'});
results.summary=summary; results.files=files; results.seeds=seeds;
writetable(summary,fullfile(root,'results','gvns_baseline_pilot_summary.csv'));
save(fullfile(root,'results','gvns_baseline_pilot_result.mat'),'results');
fprintf('GVNS baseline pilot completed: %d rows.\n',height(summary));
end


