function AnalyzeFeaturePerformance
%ANALYZEFEATUREPERFORMANCE 将事件特征与策略Gap合并，供机制诊断
scriptDir=fileparts(mfilename('fullpath'));
root=fileparts(scriptDir);
resultDir=fullfile(root,'results');
features=readtable(fullfile(resultDir,'event_features.csv'),'TextType','string');
files={fullfile(resultDir,'addition_paired_bks_gap.csv'), ...
    fullfile(resultDir,'stress_addition_bks_gap.csv')};
allRows=cell(0,1);
for f=1:numel(files)
    data=readtable(files{f},'TextType','string');
    if f==1
        dataset="addition_instances"; level=string(data.level);
    else
        dataset="addition_stress_instances"; level=string(data.stress_level);
    end
    for r=1:height(data)
        hit=features.dataset==dataset & features.level==level(r) ...
            & features.instance==data.instance(r);
        if any(hit)
            ft=features(find(hit,1),:);
            allRows{end+1,1}={dataset,level(r),data.instance(r), ...
                data.strategy(r),data.budget(r),data.gap_to_bks(r), ...
                ft.insertion_ratio,ft.mean_detour_ratio,ft.max_detour_ratio, ...
                ft.min_slack_after,ft.critical_count_after,ft.n_add}; %#ok<AGROW>
        end
    end
end
merged=cell2table(vertcat(allRows{:}),'VariableNames',{ ...
    'dataset','level','instance','strategy','budget','gap_to_bks', ...
    'insertion_ratio','mean_detour_ratio','max_detour_ratio', ...
    'min_slack','critical_count','n_add'});
writetable(merged,fullfile(resultDir,'feature_performance_merged.csv'));
save(fullfile(resultDir,'feature_performance_merged.mat'),'merged');
end
