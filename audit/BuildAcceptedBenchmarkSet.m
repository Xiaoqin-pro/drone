function BuildAcceptedBenchmarkSet
%BUILDACCEPTEDBENCHMARKSET 生成并冻结已通过可行性审计的实例集合
%   当前用于诊断验收；正式论文需要预注册生成规则后再扩大规模。

clc
clear
close all

root = fileparts(mfilename('fullpath'));
levels = {'Mild','Moderate','Severe'};
nInstances = 2;
seedBase = 3000;
maxAttempts = 5;
manifestRows = cell(0,1);

for levelIndex = 1:numel(levels)
    accepted = 0;
    attempt = 0;
    while accepted<nInstances && attempt<maxAttempts*nInstances
        attempt = attempt+1;
        scenarioSeed = seedBase+100*levelIndex+attempt;
        model = CreateModel();
        [instance,audit] = BuildFeasibleDynamicInstance( ...
            model,scenarioSeed,levels{levelIndex},2400);
        status = string({audit.status});
        isAccepted = all(status=="FEASIBLE");
        if isAccepted
            accepted = accepted+1;
            fileName = sprintf('accepted_%s_%02d.mat', ...
                levels{levelIndex},accepted);
            save(fullfile(root,'results',fileName), ...
                'instance','audit','scenarioSeed','levelIndex','accepted');
            manifestRows{end+1,1} = {string(levels{levelIndex}), ...
                accepted,scenarioSeed,attempt,fileName}; %#ok<AGROW>
            fprintf('Accepted %s instance %d: seed=%d, attempt=%d.\n', ...
                levels{levelIndex},accepted,scenarioSeed,attempt);
        else
            fprintf('Rejected %s candidate: seed=%d, statuses=%s.\n', ...
                levels{levelIndex},scenarioSeed,strjoin(status,','));
        end
    end
    if accepted<nInstances
        error('Could not build enough accepted %s instances.',levels{levelIndex});
    end
end

manifest = cell2table(vertcat(manifestRows{:}),'VariableNames',{ ...
    'level','instance','scenario_seed','attempt','file_name'});
writetable(manifest,fullfile(root,'results','accepted_benchmark_manifest.csv'));
disp(manifest);
end
