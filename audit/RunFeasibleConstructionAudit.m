function RunFeasibleConstructionAudit
%RUNFEASIBLECONSTRUCTIONAUDIT 检查可行构造动态实例能否生成参考路线
clc
clear
close all

root = fileparts(mfilename('fullpath'));
levels = {'Mild','Moderate','Severe'};
nInstances = 2;
rows = cell(0,1);

for levelIndex = 1:numel(levels)
    for instanceIndex = 1:nInstances
        scenarioSeed = 3000+instanceIndex;
        baseModel = CreateModel();
        [instance,audit] = BuildFeasibleDynamicInstance( ...
            baseModel,scenarioSeed,levels{levelIndex},2400);
        for e = 1:numel(audit)
            rows{end+1,1} = {string(levels{levelIndex}),instanceIndex, ...
                scenarioSeed,e,string(audit(e).status), ...
                audit(e).referenceCost,audit(e).referenceFE, ...
                audit(e).tightened,numel(audit(e).referenceRoute)}; %#ok<AGROW>
        end
        save(fullfile(root,'results',sprintf( ...
            'feasible_instance_%s_%02d.mat',levels{levelIndex},instanceIndex)), ...
            'instance','audit');
    end
end

summary = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'level','instance','scenario_seed','event_index','status', ...
    'reference_cost','reference_fe','tightened','route_length'});
writetable(summary,fullfile(root,'results','feasible_construction_audit.csv'));
disp(summary);
end
