function reference = ReadTSPTWReferenceSolutions(fileName)
%READTSPTWREFERENCESOLUTIONS 读取TSPTW官方best-known路线表
%   返回实例名、参考tour cost、约束违反数和客户排列。

lines = readlines(fileName);
records = struct('file',{},'cost',{},'constraintViolation',{},'permutation',{});
current = [];
for k = 1:numel(lines)
    line = strtrim(lines(k));
    if line=="" || startsWith(line,"#")
        continue;
    end
    tokens = regexp(char(line), ...
        '^([^\s]+)\s+([-+]?\d*\.?\d+)\s+(\d+)\s*(.*)$', ...
        'tokens','once');
    if ~isempty(tokens)
        if ~isempty(current)
            records(end+1) = current; %#ok<AGROW>
        end
        current.file = string(tokens{1});
        current.cost = str2double(tokens{2});
        current.constraintViolation = str2double(tokens{3});
        current.permutation = sscanf(tokens{4},'%d')';
    elseif ~isempty(current)
        continuation = sscanf(char(line),'%d')';
        current.permutation = [current.permutation,continuation]; %#ok<AGROW>
    end
end
if ~isempty(current)
    records(end+1) = current;
end

reference = records;
end
