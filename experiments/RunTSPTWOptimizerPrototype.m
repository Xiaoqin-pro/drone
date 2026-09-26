function results = RunTSPTWOptimizerPrototype
%RUNTSPTWOPTIMIZERPROTOTYPE 在原生TSPTW实例上进行离散算子公平消融
%   所有方法使用相同实例、相同随机种子和相同FE预算。

root = fileparts(fileparts(mfilename('fullpath')));
setup;
benchmarkDir = fullfile(root,'data','tsp_tw_benchmark');
manifestFile = fullfile(benchmarkDir,'manifest.csv');
if ~isfile(manifestFile), BuildTSPTWBenchmark; end
manifest = readtable(manifestFile,'Delimiter',',','TextType','string');
reference = ReadTSPTWReferenceSolutions(fullfile(root,'data','tsp_tw_raw', ...
    'best_known','SolomonPotvinBengio-best-known-traveltime.txt'));

nRuns = 10;
methods = ["RandomKeyPSO","PSO_Relocate","PSO_Swap","PSO_2Opt","PSO_Mixed"];
seeds = 1001:(1000+nRuns);
rows = cell(height(manifest)*numel(methods)*nRuns,19);
allResults = cell(height(manifest),numel(methods),nRuns);
rowIndex = 0;

for i = 1:height(manifest)
    instance = ReadTSPTWInstance(fullfile(benchmarkDir,manifest.file(i)));
    refIndex = find(strcmpi(string({reference.file}),manifest.file(i)),1);
    referenceCost = reference(refIndex).cost;
    maxFE = max(800,50*instance.nCustomers);
    baseOptions = struct('nPop',20,'maxFE',maxFE,'w',1.0,'wdamp',0.99, ...
        'c1',1.5,'c2',1.5,'latePenalty',1000,'waitPenalty',0, ...
        'localSearchFE',25,'localSearchEvery',1,'silent',true);
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
                case "PSO_Swap"
                    options.localSearchMode = 'swap';
                case "PSO_2Opt"
                    options.localSearchMode = '2opt';
                case "PSO_Mixed"
                    options.localSearchMode = 'mixed';
            end
            [best,history,stats] = RandomKeyTSPTWPSO(instance,options,[]);
            rowIndex = rowIndex+1;
            if best.Detail.isFeasible
                gap = (best.Detail.tourCost-referenceCost)/referenceCost;
            else
                gap = NaN;
            end
            rows(rowIndex,:) = {manifest.name(i),instance.nNodes, ...
                instance.nCustomers,methods(m),r,seeds(r),maxFE, ...
                stats.functionEvaluations,best.Detail.cost,best.Detail.tourCost, ...
                best.Detail.totalLate,best.Detail.isFeasible,referenceCost,gap, ...
                stats.firstFeasibleFE,stats.localSearchFE, ...
                stats.localSearchRelocateFE,stats.localSearchSwapFE, ...
                stats.localSearchTwoOptFE};
            allResults{i,m,r} = struct('best',best,'history',history, ...
                'stats',stats,'options',options,'referenceCost',referenceCost);
        end
    end
end

summary = cell2table(rows,'VariableNames', ...
    {'instance','nNodes','nCustomers','method','run','seed','maxFE', ...
    'functionEvaluations','penaltyCost','tourCost','totalLate','isFeasible', ...
    'referenceTourCost','gapToReference','firstFeasibleFE','localSearchFE', ...
    'relocateFE','swapFE','twoOptFE'});
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

fprintf('TSPTW离散算子消融完成：%d条记录。\n',height(summary));
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
            curves(r,:) = interp1([0;h.FE],[h.TourCost(1);h.TourCost], ...
                feGrid,'previous','extrap');
        end
        plot(feGrid,mean(curves,1,'omitnan'),'LineWidth',1.4, ...
            'DisplayName',results.methods(m));
    end
    grid on; xlabel('Function evaluations'); ylabel('Best feasible-first tour cost');
    title(sprintf('%s (%d customers)',results.manifest.name(i), ...
        results.manifest.nCustomers(i)),'Interpreter','none');
    legend('Location','best');
end
exportgraphics(gcf,fullfile(resultsDir,'tsptw_optimizer_prototype_anytime.png'), ...
    'Resolution',180);
close(gcf);
end
