function manifest = BuildTSPTWBenchmark
%BUILDTSPTWBENCHMARK 构建原生TSPTW开发基准清单
%   数据来源：Solomon-Potvin-Bengio TSPTW实例。
%   本脚本只复制并登记原始实例，不改写其距离矩阵和时间窗。

root = fileparts(fileparts(mfilename('fullpath')));
rawDir = fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
outDir = fullfile(root,'data','tsp_tw_benchmark');
if ~isfolder(rawDir)
    error('找不到原生TSPTW数据目录：%s',rawDir);
end
if ~isfolder(outDir), mkdir(outDir); end

files = dir(fullfile(rawDir,'*.txt'));
if isempty(files), error('原生TSPTW数据目录中没有txt实例。'); end

nNodes = zeros(numel(files),1);
for k = 1:numel(files)
    allLines = readlines(fullfile(files(k).folder,files(k).name));
    firstLine = strtrim(allLines(1));
    nNodes(k) = sscanf(char(firstLine),'%d',1);
end
valid = nNodes>=10;
files = files(valid);
nNodes = nNodes(valid);

% 先用小、中、大三个开发规模验证算法；后续正式实验再扩展到完整实例集。
targetSizes = [20,32,46];
selected = false(numel(files),1);
selectedRows = zeros(numel(targetSizes),1);
for k = 1:numel(targetSizes)
    [~,idx] = min(abs(nNodes-targetSizes(k))+selected*1e6);
    selected(idx) = true;
    selectedRows(k) = idx;
end

records = cell(numel(selectedRows),6);
for k = 1:numel(selectedRows)
    idx = selectedRows(k);
    source = fullfile(files(idx).folder,files(idx).name);
    destination = fullfile(outDir,files(idx).name);
    copyfile(source,destination);
    instance = ReadTSPTWInstance(destination);
    relativeSource = fullfile('data','tsp_tw_raw','SolomonPotvinBengio',files(idx).name);
    instanceName = string(erase(files(idx).name,'.txt'));
    records(k,:) = {instanceName,string(files(idx).name), ...
        instance.nNodes,instance.nCustomers,string(relativeSource), ...
        "Solomon-Potvin-Bengio"};
end

manifest = cell2table(records,'VariableNames', ...
    {'name','file','nNodes','nCustomers','sourceFile','source'});
writetable(manifest,fullfile(outDir,'manifest.csv'));
save(fullfile(outDir,'manifest.mat'),'manifest');

fid = fopen(fullfile(outDir,'README_CN.md'),'w');
cleanup = onCleanup(@()fclose(fid));
fprintf(fid,'# 原生 TSPTW 开发基准\n\n');
fprintf(fid,'本目录由 `experiments/BuildTSPTWBenchmark.m` 生成。\n');
fprintf(fid,'实例来自 Solomon-Potvin-Bengio TSPTW 数据集；文件内容未被重写。\n');
fprintf(fid,'当前选择三个开发规模：约20、32、46个节点（节点1为仓库）。\n');
fprintf(fid,'正式论文实验前应固定完整实例清单、随机种子和FE预算。\n');
end

