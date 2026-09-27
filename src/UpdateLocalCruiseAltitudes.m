function model = UpdateLocalCruiseAltitudes(model)
%UPDATELOCALCRUISEALTITUDES 让仓库和客户位于各自地形高度+巡航净空
if ~isfield(model,'cruiseHeight') || isempty(model.cruiseHeight), model.cruiseHeight=70; end
if isfield(model,'depotXY') && ~isempty(model.depotXY)
    depotGround=interp2(model.X,model.Y,model.terrainZ,model.depotXY(1),model.depotXY(2),'linear');
    model.depot=[model.depotXY,depotGround+model.cruiseHeight];
    model.homePosition=model.depot;
end
if isfield(model,'customerXY') && ~isempty(model.customerXY)
    customerGround=interp2(model.X,model.Y,model.terrainZ,model.customerXY(:,1),model.customerXY(:,2),'linear');
    customerAltitude=customerGround+model.cruiseHeight;
    if isfield(model,'obstacles') && ~isempty(model.obstacles)
        for q=1:size(model.customerXY,1)
            for o=1:numel(model.obstacles)
                obs=model.obstacles(o);
                inside=model.customerXY(q,1)>=obs.xMin-model.obstacleSafety && ...
                    model.customerXY(q,1)<=obs.xMax+model.obstacleSafety && ...
                    model.customerXY(q,2)>=obs.yMin-model.obstacleSafety && ...
                    model.customerXY(q,2)<=obs.yMax+model.obstacleSafety;
                if inside, customerAltitude(q)=max(customerAltitude(q),obs.zMax+model.obstacleSafety+10); end
            end
        end
    end
    model.customerXYZ=[model.customerXY,customerAltitude];
    model.customerGroundXYZ=[model.customerXY,customerGround];
end
model.flightAltitude=max([model.depot(3);model.customerXYZ(:,3)]);
if isfield(model,'obstacles') && ~isempty(model.obstacles)
    for o=1:numel(model.obstacles)
        obs=model.obstacles(o); inside=model.depotXY(1)>=obs.xMin-model.obstacleSafety && model.depotXY(1)<=obs.xMax+model.obstacleSafety && model.depotXY(2)>=obs.yMin-model.obstacleSafety && model.depotXY(2)<=obs.yMax+model.obstacleSafety;
        if inside, model.depot(3)=max(model.depot(3),obs.zMax+model.obstacleSafety+10); end
    end
end
model.nodeXYZ=[model.depot;model.customerXYZ];
end
