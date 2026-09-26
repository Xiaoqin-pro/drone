function results = RunTSPTWOptimizerPrototype
%RUNTSPTWOPTIMIZERPROTOTYPE 在原生TSPTW实例上比较三种优化器
%   1) Random-key PSO
%   2) Random-key PSO + Relocate
%   3) Random-key PSO + Relocate/Swap/2-opt
%
%   三种方法使用相同实例、相同种子和相同FE上限。

root = fileparts(fileparts(mfilename('fullpath')));
setup;
benchmarkDir = fullfile(root,'data','tsp_tw_benchmark');
manifestFile = fullfile(benchmarkDir,'manifest.csv');
if ~isfile(manifestFile)
    BuildTSPTWBenchmark;
end
manifest = readtable(manifestFile,'Delimiter',',','TextType','string');

nRuns = 3;
methods = ["RandomKeyPSO","PSO_Relocate","PSO_FullLocal"];
seeds = 1001:(1000+nRuns);
rows = cell(height(manifest)*numel(methods)*nRuns,12);
allResults = cell(height(manifest),numel(methods),nRuns);
rowIndex = 0;

for i = 1:height(manifest)
    instance = ReadTSPTWInstance(fullfile(benchmarkDir,manifest.file(i)));
    maxFE = max(800,50*instance.nCustomers);
    baseOptions = struct('nPop',20,'maxFE',maxFE,'w',1.0,'wdamp',0.99, ...
        'c1',1.5,'c2',1.5,'latePenalty',1000,'waitPenalty',0, ...
        'localSearchFE',0,'localSearchEvery',1,'silent',true);
    for m = 1:numel(methods)
        for r = 1:nRuns
            rng(seeds(r),'twister');
            options = baseOptions;
            switch methods(m)
                case "RandomKeyPSO"
                    options.localSearchMode = 'none';
                    options.localSearchFE = 0;
                case "PSO_Relocate"
                    options.localSearchMode = 'relocate';
                    options.localSearchFE = 25;
                case "PSO_FullLocal"
                    options.localSearchMode = 'full';
                    options.localSearchFE = 25;
            end
            [best,history,stats] = ...
                RandomKeyTSPTWPSO(instance,options,[]);
            rowIndex = rowIndex+1;
            rows(rowIndex,:) = {manifest.name(i),instance.nNodes, ...
                instance.nCustomers,methods(m),r,seeds(r),maxFE, ...
                stats.functionEvaluations,best.Cost,best.Detail.distance, ...
                best.Detail.totalLate,best.Detail.isFeasible};
            allResults{i,m,r} = struct('best',best,'history',history, ...
                'stats',stats,'options',options);
        end
    end
end

summary = cell2table(rows,'VariableNames', ...
    {'instance','nNodes','nCustomers','method','run','seed','maxFE', ...
    'functionEvaluations','cost','distance','totalLate','isFeasible'});
results.summary = summary;
results.runs = allResults;
results.manifest = manifest;
results.methods = methods;
results.seeds = seeds;

resultsDir = fullfile(root,'results');
if ~isfolder(resultsDir), mkdir(resultsDir); end
writetable(summary,fullfile(resultsDir,'tsptw_optimizer_prototype_summary.csv'));
save(fullfile(resultsDir,'tsptw_optimizer_prototype_result.mat'),'results','-v7.3');
PlotPrototypeCurves(results,resultsDir);

fprintf('TSPTW优化器原型实验完成：%d条记录。\n',height(summary));
fprintf('结果：%s\n',fullfile(resultsDir,'tsptw_optimizer_prototype_summary.csv'));
end

function PlotPrototypeCurves(results,resultsDir)
figure('Visible','off','Color','w');
tiledlayout(height(results.manifest),1,'Padding','compact','TileSpacing','compact');
for i = 1:height(results.manifest)
    nexttile; hold on;
    for m = 1:numel(results.methods)
        histories = results.runs(i,m,:);
        finalFE = cellfun(@(x)x.history.FE(end),histories);
        feGrid = linspace(0,max(finalFE),100);
        curves = nan(numel(histories),numel(feGrid));
        for r = 1:numel(histories)
            h = histories{r}.history;
            curves(r,:) = interp1([0;h.FE],[h.Cost(1);h.Cost], ...
                feGrid,'previous','extrap');
        end
        plot(feGrid,mean(curves,1,'omitnan'),'LineWidth',1.5, ...
            'DisplayName',results.methods(m));
    end
    grid on; xlabel('Function evaluations'); ylabel('Best cost');
    title(sprintf('%s (%d customers)',results.manifest.name(i), ...
        results.manifest.nCustomers(i)),'Interpreter','none');
    legend('Location','best');
end
exportgraphics(gcf,fullfile(resultsDir,'tsptw_optimizer_prototype_anytime.png'), ...
    'Resolution',180);
close(gcf);
end



