function routeIDs = DecodeRoute(position,activeCustomerIDs)
%DECODEROUTE 将连续优先级键解码为活动客户的访问顺序
[~,order] = sort(position,'ascend');
activeCustomerIDs = activeCustomerIDs(:)';
routeIDs = activeCustomerIDs(order);
end
