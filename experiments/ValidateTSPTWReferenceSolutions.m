function report = ValidateTSPTWReferenceSolutions
%VALIDATETSPTWREFERENCESOLUTIONS 用官方路线回归测试TSPTW读取器和评价器
%   只验证travel-time参考表；makespan表作为后续动态扩展的独立参考。

root = fileparts(fileparts(mfilename('fullpath')));
setup;
manifestFile = fullfile(root,'data','tsp_tw_benchmark','manifest.csv');
if ~isfile(manifestFile)
    BuildTSPTWBenchmark;
end
manifest = readtable(manifestFile,'Delimiter',',','TextType','string');
referenceFile = fullfile(root,'data','tsp_tw_raw','best_known', ...
    'SolomonPotvinBengio-best-known-traveltime.txt');
reference = ReadTSPTWReferenceSolutions(referenceFile);

rows = cell(height(manifest),8);
for k = 1:height(manifest)
    name = manifest.file(k);
    idx = find(strcmpi(string({reference.file}),name),1);
    if isempty(idx)
        error('参考文件中没有实例：%s',name);
    end
    instance = ReadTSPTWInstance(fullfile(root,'data','tsp_tw_benchmark',name));
    permutation = reference(idx).permutation;
    if numel(permutation)~=instance.nCustomers
        error('%s参考排列长度错误。',name);
    end
    route = permutation+1;
    [~,detail] = EvaluateTSPTWRoute(route,instance,struct('latePenalty',1000));
    costError = detail.tourCost-reference(idx).cost;
    pass = detail.totalLate<=1e-8 && abs(costError)<=1e-2 ...
        && reference(idx).constraintViolation==0;
    rows(k,:) = {name,instance.nCustomers,reference(idx).cost, ...
        detail.tourCost,detail.totalLate,costError,pass, ...
        string(strjoin(string(permutation),' '))};
end

report = cell2table(rows,'VariableNames', ...
    {'instance','nCustomers','referenceTourCost','evaluatedTourCost', ...
    'totalLate','costError','pass','permutation'});
resultsDir = fullfile(root,'results');
if ~isfolder(resultsDir), mkdir(resultsDir); end
writetable(report,fullfile(resultsDir,'tsptw_reference_validation.csv'));
save(fullfile(resultsDir,'tsptw_reference_validation.mat'),'report');
if ~all(report.pass)
    error('TSPTW官方参考路线校验失败。');
end
fprintf('TSPTW官方参考路线校验通过：%d个实例。\n',height(report));
end
