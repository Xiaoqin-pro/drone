function data = ReadSolomonInstance(fileName)
%READSOLOMONINSTANCE 读取Solomon VRPTW文本实例和车辆元数据
lines = readlines(fileName);
lines = strtrim(lines);
data.name = string(strtrim(lines(1)));
vehicleHeader = find(contains(upper(lines),'NUMBER') & contains(upper(lines),'CAPACITY'),1);
vehicleLine = sscanf(char(lines(vehicleHeader+1)),'%f');
if numel(vehicleLine)>=2
    data.vehicleNumber = vehicleLine(1); data.capacity = vehicleLine(2);
else
    data.vehicleNumber = NaN; data.capacity = NaN;
end
headerIndex = find(contains(upper(lines),'CUST NO.'),1);
if isempty(headerIndex), error('Solomon customer header not found: %s',fileName); end
rows = zeros(0,7);
for k=headerIndex+1:numel(lines)
    values=sscanf(char(lines(k)),'%f');
    if numel(values)>=7, rows(end+1,:)=values(1:7)'; end %#ok<AGROW>
end
if isempty(rows), error('No customer rows found.'); end
data.id=rows(:,1); data.xy=rows(:,2:3); data.demand=rows(:,4);
data.readyTime=rows(:,5); data.dueTime=rows(:,6); data.service=rows(:,7);
data.depotRow=rows(1,:); data.customers=rows(2:end,:); data.customerCount=size(data.customers,1);
end
