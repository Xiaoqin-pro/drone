function instance = GenerateDynamicInstance(model,scenarioSeed,level)
%GENERATEDYNAMICINSTANCE 由scenarioSeed生成一个可复现的配对动态实例
%   场景随机性与算法随机性分离：本函数只决定事件内容。

rng(scenarioSeed,'twister');
level = char(level);

switch lower(level)
    case 'mild'
        nCancel = 1; nAdd = 1;
    case 'moderate'
        nCancel = 2; nAdd = 2;
    case 'severe'
        nCancel = 4; nAdd = 4;
    otherwise
        error('Unknown event magnitude: %s',level);
end

% 避免优先取消最早可能已经完成的客户。
eligible = model.customerIDs(6:end)';
cancelIDs = eligible(randperm(numel(eligible),nCancel));
addIDs = (model.nCustomers+1):(model.nCustomers+nAdd);

for k = 1:nAdd
    id = addIDs(k);
    xy = [100+800*rand,100+600*rand];
    earliest = 120+25*rand;
    latest = earliest+220+40*rand;
    addCustomers(k).id = id; %#ok<AGROW>
    addCustomers(k).xy = xy;
    addCustomers(k).window = [earliest latest];
    addCustomers(k).service = 8+4*rand;
end

events(1).time = 90;
events(1).type = 'cancel';
events(1).customerIDs = sort(cancelIDs);
events(2).time = 150;
events(2).type = 'add';
events(2).customerIDs = addIDs;
events(2).customers = addCustomers;

instance.scenarioSeed = scenarioSeed;
instance.level = string(level);
instance.cancelIDs = cancelIDs;
instance.addIDs = addIDs;
instance.events = events;
end
