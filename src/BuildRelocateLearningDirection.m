function [direction,detail] = BuildRelocateLearningDirection(routeBefore,routeAfter,customerIDs)
%BUILDRELOCATELEARNINGDIRECTION 将离散Relocate前后排列差异编码为random-key方向
%   direction可直接作为PSO连续速度中的全局反馈项。

routeBefore=routeBefore(:)'; routeAfter=routeAfter(:)'; customerIDs=customerIDs(:)';
keysBefore=RouteToKeys(routeBefore,customerIDs);
keysAfter=RouteToKeys(routeAfter,customerIDs);
direction=keysAfter-keysBefore;
scale=max(norm(direction),eps);
direction=direction/scale;
changed=find(abs(keysAfter-keysBefore)>1e-12);
detail.changedCustomerIDs=customerIDs(changed);
detail.rawKeyDelta=keysAfter-keysBefore;
detail.norm=norm(keysAfter-keysBefore);
detail.changedCount=numel(changed);
end

function keys=RouteToKeys(route,customerIDs)
keys=zeros(1,numel(customerIDs));
for k=1:numel(route)
    idx=find(customerIDs==route(k),1);
    if ~isempty(idx), keys(idx)=k/max(numel(route),1); end
end
end
