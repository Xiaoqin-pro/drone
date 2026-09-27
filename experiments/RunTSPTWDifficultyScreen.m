function results = RunTSPTWDifficultyScreen
%RUNTSPTWDifficultySCREEN 用LateOnly扫描全部SPB TSPTW实例并分层

root=fileparts(fileparts(mfilename('fullpath'))); setup;
rawDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
files=dir(fullfile(rawDir,'*.txt')); seeds=1:10; rows=cell(0,1);
for i=1:numel(files)
    try
        instance=ReadTSPTWInstance(fullfile(files(i).folder,files(i).name));
    catch
        continue;
    end
    maxFE=100*instance.nCustomers; feasible=zeros(numel(seeds),1); first=nan(numel(seeds),1); late=zeros(numel(seeds),1);
    for s=1:numel(seeds)
        rng(991000+100*i+s,'twister');
        options=struct('nPop',20,'maxFE',maxFE,'localSearchMode','late', ...
            'localSearchFE',max(1,instance.nCustomers-1),'latePenalty',1000,'silent',true);
        [best,~,stats]=RandomKeyTSPTWPSO(instance,options,[]);
        feasible(s)=best.Detail.isFeasible; late(s)=best.Detail.totalLate;
        if isfinite(stats.firstFeasibleFE), first(s)=stats.firstFeasibleFE; end
    end
    rate=mean(feasible); validFirst=first(isfinite(first));
    if rate>=0.8, tier="easy"; elseif rate>=0.2, tier="hard-feasible-transition"; else, tier="very-hard"; end
    rows{end+1,1}={string(files(i).name),instance.nCustomers,rate, ...
        median(validFirst,'omitnan'),median(late),tier}; %#ok<AGROW>
    fprintf('%s n=%d feasible=%.2f tier=%s\n',files(i).name,instance.nCustomers,rate,tier);
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','nCustomers','feasible_rate','median_first_feasible_fe', ...
    'median_total_late','tier'});
results.summary=summary; results.seeds=seeds;
writetable(summary,fullfile(root,'results','tsptw_difficulty_screen_summary.csv'));
save(fullfile(root,'results','tsptw_difficulty_screen_result.mat'),'results');
fprintf('Difficulty screen completed: %d instances.\n',height(summary));
end
