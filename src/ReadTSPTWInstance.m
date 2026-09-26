function instance = ReadTSPTWInstance(fileName)
%READTSPTWINSTANCE 读取 Solomon-Potvin-Bengio TSPTW实例
%   文件格式：
%   第1行：节点数 n（包含仓库节点）
%   接下来 n 行：n×n 的旅行代价矩阵
%   最后 n 行：每个节点的 ready time 和 due time
%
%   节点1默认作为仓库，节点2:n为必须访问的客户。

if nargin<1 || ~(ischar(fileName) || isstring(fileName))
    error('fileName必须是文本文件路径。');
end

fileName = char(fileName);
if ~isfile(fileName)
    error('找不到TSPTW实例文件：%s',fileName);
end

lines = readlines(fileName);
lines = strtrim(lines);
lines(lines=="") = [];
if isempty(lines)
    error('TSPTW实例文件为空：%s',fileName);
end

n = sscanf(char(lines(1)),'%d',1);
if isempty(n) || n<2
    error('TSPTW实例第一行必须是至少为2的节点数。');
end

requiredLines = 1+n+n;
if numel(lines)<requiredLines
    error('TSPTW实例行数不足：期望至少%d行，实际%d行。', ...
        requiredLines,numel(lines));
end

costMatrix = zeros(n,n);
for i = 1:n
    values = sscanf(char(lines(1+i)),'%f');
    if numel(values)<n
        error('第%d个代价矩阵行的字段不足。',i);
    end
    costMatrix(i,:) = values(1:n)';
end

windows = zeros(n,2);
service = zeros(n,1);
for i = 1:n
    values = sscanf(char(lines(1+n+i)),'%f');
    if numel(values)<2
        error('第%d个时间窗行的字段不足。',i);
    end
    windows(i,:) = values(1:2)';
    if numel(values)>=3
        service(i) = values(3);
    end
end

instance.name = string(erase(string(fileName),[".txt",".dat"]));
instance.fileName = string(fileName);
instance.nNodes = n;
instance.nCustomers = n-1;
instance.depotIndex = 1;
instance.customerIDs = (2:n)';
instance.costMatrix = costMatrix;
instance.windows = windows;
instance.service = service;
instance.symmetricCost = max(max(abs(costMatrix-costMatrix'))) < 1e-9;
instance.distanceScale = max(max(costMatrix));
end

