function results = RunCompetitiveNeighborhoodForkAudit
%RUNCOMPETITIVENEIGHBORHOODFORKAUDIT 从同一first-feasible路线比较2opt/VND/Competitive
root=fileparts(fileparts(mfilename('fullpath'))); setup;
rawDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
selected=["rc_201.1.txt","rc_201.2.txt","rc_201.3.txt","rc_202.1.txt", ...
    "rc_202.3.txt","rc_204.2.txt","rc_207.3.txt","rc_204.1.txt","rc_208.3.txt"];
methods=["2opt","VND","Competitive"]; seeds=1:10; rows=cell(0,1);
for i=1:numel(selected)
    instance=ReadTSPTWInstance(fullfile(rawDir,selected(i))); n=instance.nCustomers;
    q=max(1,n-1); firstBudget=100*n; forkBudget=3*q;
    for seedIndex=1:numel(seeds)
        seed=998000+100*i+seedIndex; rng(seed,'twister');
        baseOptions=struct('nPop',20,'maxFE',firstBudget,'localSearchMode','late', ...
            'localSearchFE',q,'restorationFE',q,'latePenalty',1000,'silent',true);
        [~,~,baseStats]=RandomKeyTSPTWPSO(instance,baseOptions,[]);
        if isempty(baseStats.firstFeasibleRoute), continue; end
        route=baseStats.firstFeasibleRoute; detail=baseStats.firstFeasibleDetail;
        for m=1:numel(methods)
            rng(seed+50000*m,'twister');
            if methods(m)=="2opt"
                opt=struct('neighborhoodOrder',"2opt",'neighborhoodQuota',q,'latePenalty',1000);
                [~,afterDetail,localStats]=BudgetedVND(route,instance,q,detail,opt);
            elseif methods(m)=="VND"
                opt=struct('neighborhoodOrder',["2opt","swap","relocate"], ...
                    'neighborhoodQuota',q,'latePenalty',1000);
                [~,afterDetail,localStats]=BudgetedVND(route,instance,forkBudget,detail,opt);
            else
                [~,afterDetail,localStats]=CompetitiveNeighborhoodSearch(route,instance, ...
                    forkBudget,detail,struct('latePenalty',1000));
            end
            gain=detail.tourCost-afterDetail.tourCost;
            rows{end+1,1}={selected(i),n,seed,methods(m),baseStats.firstFeasibleFE, ...
                detail.tourCost,afterDetail.tourCost,gain,gain/max(localStats.functionEvaluations,1), ...
                localStats.functionEvaluations}; %#ok<AGROW>
        end
    end
    fprintf('%s completed.\n',selected(i));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','nCustomers','seed','method','first_feasible_fe','first_cost', ...
    'final_cost','post_gain','gain_per_fe','local_search_fe'});
results.summary=summary; results.methods=methods;
writetable(summary,fullfile(root,'results','competitive_neighborhood_fork_summary.csv'));
save(fullfile(root,'results','competitive_neighborhood_fork_result.mat'),'results');
fprintf('Competitive neighborhood fork completed: %d rows.\n',height(summary));
end
