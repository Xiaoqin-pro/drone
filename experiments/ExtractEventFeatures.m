function ExtractEventFeatures
%EXTRACTEVENTFEATURES 提取事件发生时可计算的结构影响特征
scriptDir = fileparts(mfilename('fullpath'));
root = fileparts(scriptDir);
resultDir = fullfile(root,'results');
datasets = {'addition_instances','addition_stress_instances'};
rows = cell(0,1);

for d=1:numel(datasets)
    dataDir = fullfile(resultDir,datasets{d});
    manifestFile = dir(fullfile(dataDir,'*manifest.csv'));
    if isempty(manifestFile), continue; end
    manifest = readtable(fullfile(dataDir,manifestFile(1).name), ...
        'TextType','string');
    if ismember('level',manifest.Properties.VariableNames)
        levelColumn = string(manifest.level);
    else
        levelColumn = string(manifest.stress_level);
    end

    % 基础环境和事件前参考路线只计算一次。
    baseModel = CreateModel();
    baseModel.windows(:,2) = baseModel.windows(:,2)+300;
    baseModel.cfg.silent = true;
    baseIDs = baseModel.customerIDs(:)';
    baseCache = BuildLegCache(baseModel,baseIDs,baseModel.homePosition);
    rng(860000,'twister');
    [baseSolution,~,~] = RoutingPSO(baseModel,baseCache,100,40, ...
        0.90,0.995,1.7,1.7,[]);

    for r=1:height(manifest)
        loaded = load(fullfile(dataDir,manifest.file_name(r)));
        instance = loaded.instance;
        model = instance.model;
        event = instance.event;
        route = instance.referenceRoute(:)';
        newIDs = event.customerIDs(:)';
        originalIDs = 1:20;
        oldActive = setdiff(originalIDs,instance.state.completedIDs,'stable');
        oldRoute = baseSolution.Detail.routeIDs(ismember( ...
            baseSolution.Detail.routeIDs,oldActive));

        oldModel = baseModel;
        oldModel.startTime = instance.state.time;
        oldModel.depot = instance.state.position;
        oldCache = BuildLegCache(oldModel,oldActive,instance.state.position);
        [oldCost,oldDetail] = EvaluateSchedule(oldRoute,oldModel,oldCache);

        % 从事件前旧路线开始，逐个做最优直接插入。
        directRoute = oldRoute;
        for id = newIDs
            bestCost = inf;
            bestRoute = directRoute;
            for pos=1:numel(directRoute)+1
                candidate = [directRoute(1:pos-1),id,directRoute(pos:end)];
                [candidateCost,~] = EvaluateSchedule(candidate,model,instance.cache);
                if candidateCost<bestCost
                    bestCost = candidateCost;
                    bestRoute = candidate;
                end
            end
            directRoute = bestRoute;
        end
        [directCost,directDetail] = EvaluateSchedule(directRoute,model,instance.cache);

        deltaInsertion = directDetail.distance-oldDetail.distance;
        insertionRatio = deltaInsertion/max(oldDetail.distance,eps);
        detour = zeros(0,1);
        for id=newIDs
            pos=find(directRoute==id,1);
            localNode=find(instance.cache.customerIDs==id,1)+1;
            if pos==1
                from=instance.cache.nodes(instance.cache.startIndex,:);
            else
                prevID=directRoute(pos-1);
                prevNode=find(instance.cache.customerIDs==prevID,1)+1;
                from=instance.cache.nodes(prevNode,:);
            end
            if pos==numel(directRoute)
                to=instance.cache.nodes(instance.cache.homeIndex,:);
            else
                nextID=directRoute(pos+1);
                nextNode=find(instance.cache.customerIDs==nextID,1)+1;
                to=instance.cache.nodes(nextNode,:);
            end
            if pos==1
                legIn=instance.cache.legs{instance.cache.startIndex,localNode}.distance;
            else
                legIn=instance.cache.legs{prevNode,localNode}.distance;
            end
            if pos==numel(directRoute)
                legOut=instance.cache.legs{localNode,instance.cache.homeIndex}.distance;
            else
                legOut=instance.cache.legs{localNode,nextNode}.distance;
            end
            detour(end+1,1)=(legIn/max(norm(from-instance.cache.nodes(localNode,:)),eps) ...
                +legOut/max(norm(instance.cache.nodes(localNode,:)-to),eps))/2; %#ok<AGROW>
        end

        [minSlackBefore,meanSlackBefore] = SlackStats(oldDetail,oldModel);
        [minSlackAfter,meanSlackAfter] = SlackStats(directDetail,model);
        commonIDs=intersect(oldRoute,directRoute,'stable');
        slackLoss=0;
        if ~isempty(commonIDs)
            before=SlackVector(oldDetail,oldModel,commonIDs);
            after=SlackVector(directDetail,model,commonIDs);
            slackLoss=mean(max(0,before-after));
        end
        rows{end+1,1}={string(datasets{d}),levelColumn(r),manifest.instance(r), ...
            manifest.scenario_seed(r),numel(newIDs),oldCost,directCost, ...
            deltaInsertion,insertionRatio,mean(detour),max(detour), ...
            minSlackBefore,minSlackAfter,meanSlackBefore,meanSlackAfter, ...
            slackLoss,sum(SlackVector(directDetail,model,directRoute)<30), ...
            instance.referenceCost}; %#ok<AGROW>
    end
end

features=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'dataset','level','instance','scenario_seed','n_add','old_cost', ...
    'direct_repair_cost','delta_insertion_distance','insertion_ratio', ...
    'mean_detour_ratio','max_detour_ratio','min_slack_before', ...
    'min_slack_after','mean_slack_before','mean_slack_after','slack_loss', ...
    'critical_count_after','witness_cost'});
writetable(features,fullfile(resultDir,'event_features.csv'));
save(fullfile(resultDir,'event_features.mat'),'features');
end

function [minSlack,meanSlack]=SlackStats(detail,model)
slack=SlackVector(detail,model,detail.routeIDs);
minSlack=min(slack); meanSlack=mean(slack);
end

function slack=SlackVector(detail,model,ids)
slack=zeros(1,numel(ids));
for k=1:numel(ids)
    hit=find(detail.records(:,1)==ids(k),1);
    if isempty(hit), slack(k)=NaN; else, slack(k)=model.windows(ids(k),2)-detail.records(hit,3); end
end
slack=slack(~isnan(slack));
end
