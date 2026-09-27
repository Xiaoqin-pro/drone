function results = RunCompetitiveNeighborhoodForkAuditV2
%RUNCOMPETITIVENEIGHBORHOODFORKAUDITV2 同一可行路线、相同FE上限和随机流的fork
%   Repeated2Opt、VND和Competitive的FE上限均为3*(n-1)。
%   局部停滞可提前结束，因此同时报告实际消耗FE，不能把上限当作实际FE。

root=fileparts(fileparts(mfilename('fullpath'))); setup;
rawDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
selected=["rc_201.1.txt","rc_201.2.txt","rc_201.3.txt","rc_202.1.txt", ...
    "rc_202.3.txt","rc_204.2.txt","rc_207.3.txt","rc_204.1.txt","rc_208.3.txt"];
methods=["Repeated2Opt","BudgetedVND","Competitive"];
seeds=1:10; rows=cell(0,1);
for i=1:numel(selected)
    instance=ReadTSPTWInstance(fullfile(rawDir,selected(i)));
    q=max(1,instance.nCustomers-1); postBudget=3*q;
    for seedIndex=1:numel(seeds)
        seed=998000+100*i+seedIndex;
        rng(seed,'twister');
        options=struct('nPop',20,'maxFE',100*instance.nCustomers, ...
            'localSearchMode','late','localSearchFE',q,'latePenalty',1000,'silent',true);
        [~,~,baseStats]=RandomKeyTSPTWPSO(instance,options,[]);
        if isempty(baseStats.firstFeasibleRoute), continue; end
        route=baseStats.firstFeasibleRoute; detail=baseStats.firstFeasibleDetail;
        forkSeed=seed+50000;
        for m=1:numel(methods)
            rng(forkSeed,'twister');
            searchOptions=struct('latePenalty',1000,'neighborhoodQuota',q);
            if methods(m)=="Repeated2Opt"
                searchOptions.neighborhoodOrder="2opt";
                [~,afterDetail,localStats]=BudgetedVND(route,instance, ...
                    postBudget,detail,searchOptions);
            elseif methods(m)=="BudgetedVND"
                searchOptions.neighborhoodOrder=["2opt","swap","relocate"];
                [~,afterDetail,localStats]=BudgetedVND(route,instance, ...
                    postBudget,detail,searchOptions);
            else
                [~,afterDetail,localStats]=CompetitiveNeighborhoodSearch( ...
                    route,instance,postBudget,detail,searchOptions);
            end
            gain=detail.tourCost-afterDetail.tourCost;
            rows{end+1,1}={selected(i),instance.nCustomers,seed,forkSeed, ...
                methods(m),baseStats.firstFeasibleFE,postBudget, ...
                localStats.functionEvaluations,detail.tourCost, ...
                afterDetail.tourCost,gain,gain/max(localStats.functionEvaluations,1), ...
                afterDetail.isFeasible}; %#ok<AGROW>
        end
    end
    fprintf('%s fork V2 completed.\n',selected(i));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','nCustomers','seed','fork_seed','method','first_feasible_fe', ...
    'post_fe_cap','actual_post_fe','first_cost','final_cost','post_gain', ...
    'gain_per_fe','is_feasible'});
results.summary=summary; results.methods=methods; results.selected=selected;
writetable(summary,fullfile(root,'results','competitive_neighborhood_fork_v2.csv'));
save(fullfile(root,'results','competitive_neighborhood_fork_v2.mat'),'results');
fprintf('Fork V2 completed: %d rows.\n',height(summary));
end
