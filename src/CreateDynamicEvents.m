function events = CreateDynamicEvents(model)
%CREATEDYNAMICEVENTS 创建可扩展的动态订单事件序列
%   第一阶段包含订单取消和新增订单两个事件。

events(1).time = 90;
events(1).type = 'cancel';
events(1).customerID = 7;

events(2).time = 150;
events(2).type = 'add';
events(2).customer.id = model.nCustomers+1;
events(2).customer.xy = [610 140];
events(2).customer.window = [145 310];
events(2).customer.service = 10;
end
