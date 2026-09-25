function model = ApplyDynamicEvent(model,event)
%APPLYDYNAMICEVENT 更新任务集合和客户数据
switch lower(event.type)
    case 'cancel'
        % 删除动作由主循环更新activeIDs完成，这里只保留原始任务数据。

    case 'add'
        id = event.customer.id;
        xy = event.customer.xy;
        groundZ = interp2(model.X,model.Y,model.terrainZ, ...
            xy(1),xy(2),'linear');
        model.customerXY(id,:) = xy;
        model.customerXYZ(id,:) = [xy,model.flightAltitude];
        model.customerGroundXYZ(id,:) = [xy,groundZ];
        model.windows(id,:) = event.customer.window;
        model.service(id,1) = event.customer.service;
        model.customerIDs = (1:size(model.customerXY,1))';
        model.nCustomers = numel(model.customerIDs);

    otherwise
        error('Unknown dynamic event type: %s',event.type);
end
end
