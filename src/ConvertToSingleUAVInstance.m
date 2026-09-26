function model = ConvertToSingleUAVInstance(solomon,baseModel,selectedRows,use3D)
%CONVERTTOSINGLEUAVINSTANCE 将Solomon子集转换为单UAV模型
if nargin<4, use3D=false; end
rows=solomon.customers(selectedRows,:);
scale=8; offset=[100 0];
customerXY=[offset(1)+scale*rows(:,2),offset(2)+scale*rows(:,3)];
depotXY=[offset(1)+scale*solomon.depotRow(2),offset(2)+scale*solomon.depotRow(3)];
model=baseModel;
model.depotXY=depotXY; model.customerXY=customerXY;
model.nCustomers=size(rows,1); model.customerIDs=(1:model.nCustomers)'; model.startTime=0;
model.customerSourceID=rows(:,1); model.windows=rows(:,5:6); model.service=rows(:,7); model.demand=rows(:,4);
model.sourceName=solomon.name; model.sourceScale=scale; model.sourceOffset=offset; model.use3D=use3D;
if use3D
    model.cruiseHeight=70; model.flightAltitude=max(model.terrainZ(:))+model.cruiseHeight;
    depotGround=interp2(model.X,model.Y,model.terrainZ,depotXY(1),depotXY(2),'linear');
    customerGround=interp2(model.X,model.Y,model.terrainZ,customerXY(:,1),customerXY(:,2),'linear');
    model.depot=[depotXY,model.flightAltitude]; model.homePosition=model.depot;
    model.customerXYZ=[customerXY,repmat(model.flightAltitude,model.nCustomers,1)];
    model.customerGroundXYZ=[customerXY,customerGround]; model.nodeXYZ=[model.depot;model.customerXYZ];
    model.speed=scale;
else
    model.depot=[depotXY,0]; model.homePosition=model.depot;
    model.customerXYZ=[customerXY,zeros(model.nCustomers,1)]; model.customerGroundXYZ=model.customerXYZ;
    model.nodeXYZ=[model.depot;model.customerXYZ]; model.terrainZ=zeros(size(model.terrainZ));
    model.obstacles=repmat(model.obstacles,0,1); model.nObstacles=0; model.speed=scale;
    model.minClearance=0; model.terrainPenalty=0; model.obstaclePenalty=0;
end
end
