function RunRestartLongDiagnostic
%RUNRESTARTLONGDIAGNOSTIC 对8个代表实例做Restart 20k FE诊断
scriptDir = fileparts(mfilename('fullpath'));
root = fileparts(scriptDir);
resultDir = fullfile(root,'results');
checkpoints = [500 1000 1500 2500 5000 10000 20000];
population = 60;
maxFE = max(checkpoints);
maxIt = floor(maxFE/population)-1;

cases = {
    'addition_instances','M1',1;
    'addition_instances','M4',1;
    'addition_stress_instances','Near',1;
    'addition_stress_instances','Near',2;
    'addition_stress_instances','Far',1;
    'addition_stress_instances','Far',2;
    'addition_stress_instances','Tight',1;
    'addition_stress_instances','Tight',2};

bks = readtable(fullfile(resultDir,'offline_bks_summary_updated.csv'), ...
    'TextType','string');
rows = cell(0,1);
curves = cell(size(cases,1),1);

for c = 1:size(cases,1)
    dataset = cases{c,1};
    level = cases{c,2};
    instanceID = cases{c,3};
    manifest = readtable(fullfile(resultDir,dataset, ...
        GetManifestName(dataset)),'TextType','string');
    if strcmp(dataset,'addition_instances')
        hit = string(manifest.level)==string(level) ...
            & manifest.instance==instanceID;
    else
        hit = string(manifest.stress_level)==string(level) ...
            & manifest.instance==instanceID;
    end
    fileName = manifest.file_name(find(hit,1));
    loaded = load(fullfile(resultDir,dataset,fileName));
    instance = loaded.instance;
    model = instance.model;
    model.cfg.silent = true;
    bksRow = bks.dataset==string(dataset) & bks.level==string(level) ...
        & bks.instance==instanceID;
    bksCost = bks.bks_cost(find(bksRow,1));

    rng(990000+c,'twister');
    [~,~,stats,history] = RoutingPSO(model,instance.cache,maxIt, ...
        population,0.90,0.995,1.7,1.7,[]);
    curves{c} = history;

    for k = 1:numel(checkpoints)
        index = find(history.FE<=checkpoints(k),1,'last');
        if isempty(index)
            continue;
        end
        rows{end+1,1} = {string(dataset),string(level),instanceID, ...
            checkpoints(k),history.FE(index),history.Distance(index), ...
            history.Late(index),history.ObstacleViolation(index), ...
            history.IsFeasible(index), ...
            (history.Distance(index)-bksCost)/max(abs(bksCost),eps), ...
            bksCost,stats.functionEvaluations}; %#ok<AGROW>
    end
    fprintf('%s %s %d completed.\n',dataset,level,instanceID);
end

summary = cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'dataset','level','instance','checkpoint_fe','actual_fe', ...
    'distance','late','obstacle_violation','is_feasible', ...
    'gap_to_bks','bks_cost','run_total_fe'});
writetable(summary,fullfile(resultDir,'restart_long_summary.csv'));
save(fullfile(resultDir,'restart_long_result.mat'), ...
    'summary','curves','cases','checkpoints','maxFE','population');

f = figure('Visible','off','Color','w');
for c=1:size(cases,1)
    subplot(2,4,c);
    h=curves{c};
    bksRow=bks.dataset==string(cases{c,1}) ...
        & bks.level==string(cases{c,2}) & bks.instance==cases{c,3};
    bksCost=bks.bks_cost(find(bksRow,1));
    plot(h.FE,(h.Distance-bksCost)/max(abs(bksCost),eps),'LineWidth',1.6);
    yline(0,'k--'); grid on;
    title(sprintf('%s-%s-%d',cases{c,1},cases{c,2},cases{c,3}), ...
        'Interpreter','none');
    xlabel('FE'); ylabel('Gap');
end
exportgraphics(f,fullfile(resultDir,'restart_long_gap_curves.png'), ...
    'Resolution',180);
end

function name = GetManifestName(dataset)
if strcmp(dataset,'addition_instances')
    name = 'addition_manifest.csv';
else
    name = 'stress_manifest.csv';
end
end
