function data = ReadSolomonInstance(fileName)
%READSOLOMONINSTANCE 读取Solomon VRPTW文本实例
lines = readlines(fileName);
lines = strtrim(lines);
headerIndex = find(contains(upper(lines),'CUST NO.'),1);
if isempty(headerIndex)
    error('Solomon customer header not found: %s',fileName);
end
rows = [];
for k=headerIndex+1:numel(lines)
    line=char(lines(k));
    if isempty(strtrim(line)), continue; end
    values=sscanf(line,'%f');
    if numel(values)>=7
        rows(end+1,:)=values(1:7)'; %#ok<AGROW>
    end
end
if isempty(rows), error('No customer rows found.'); end

data.name = string(strtrim(lines(1)));
data.vehicleNumber = sscanf(char(lines(headerIndex-2)),'%f');
data.capacity = sscanf(char(lines(headerIndex-1)),'%f');
data.id = rows(:,1);
data.xy = rows(:,2:3);
data.demand = rows(:,4);
data.readyTime = rows(:,5);
data.dueTime = rows(:,6);
data.service = rows(:,7);
data.depotRow = rows(1,:);
data.customers = rows(2:end,:);
data.customerCount = size(data.customers,1);
end
