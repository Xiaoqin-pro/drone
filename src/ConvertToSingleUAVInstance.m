function model = ConvertToSingleUAVInstance(solomon,baseModel,nCustomers)
%CONVERTTOSINGLEUAVINSTANCE 将Solomon客户数据转换为单UAV模型
if nargin<3 || isempty(nCustomers)
    nCustomers = min(20,solomon.customerCount);
end
rows = solomon.customers(1:nCustomers,:);
scaleX = baseModel.mapSize(1)/100;
scaleY = baseModel.mapSize(2)/100;
customerXY = [rows(:,2)*scaleX,rows(:,3)*scaleY];
depotXY = [solomon.depotRow(2)*scaleX,solomon.depotRow(3)*scaleY];

model = baseModel;
model.depotXY = depotXY;
model.customerXY = customerXY;
model.nCustomers = nCustomers;
model.customerIDs = (1:nCustomers)';
model.startTime = 0;
model.customerSourceID = rows(:,1);
model.windows = rows(:,5:6);
model.service = rows(:,7);
model.demand = rows(:,4);
model.sourceName = solomon.name;
model.sourceScale = [scaleX scaleY];

model.cruiseHeight = 70;
model.flightAltitude = max(model.terrainZ(:))+model.cruiseHeight;
depotGroundZ = interp2(model.X,model.Y,model.terrainZ, ...
    depotXY(1),depotXY(2),'linear');
customerGroundZ = interp2(model.X,model.Y,model.terrainZ, ...
    customerXY(:,1),customerXY(:,2),'linear');
model.depot = [depotXY,model.flightAltitude];
model.homePosition = model.depot;
model.customerXYZ = [customerXY,repmat(model.flightAltitude,nCustomers,1)];
model.customerGroundXYZ = [customerXY,customerGroundZ];
model.nodeXYZ = [model.depot;model.customerXYZ];
end
