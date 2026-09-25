function ExtractEventFeatures
%EXTRACTEVENTFEATURES 提取已有新增订单实例的真实事件影响特征
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
    for r=1:height(manifest)
        loaded = load(fullfile(dataDir,manifest.file_name(r)));
        instance = loaded.instance;
        model = instance.model;
        route = instance.referenceRoute(:)';
        newIDs = instance.event.customerIDs(:)';
        oldIDs = setdiff(instance.activeIDs,newIDs,'stable');
        newIDs = intersect(newIDs,route,'stable');
        [referenceCost,referenceDetail] = EvaluateSchedule(route,model,instance.cache);
        baseRoute = route(ismember(route,oldIDs));
        baseCost = 0;
        baseDistance = 0;
        if ~isempty(baseRoute)
            [baseCost,baseDetail] = EvaluateSchedule(baseRoute,model,instance.cache);
            baseDistance = baseDetail.distance;
        else
            baseCost = 0;
            baseDistance = max(referenceDetail.distance,eps);
        end
        if ~exist('baseDistance','var')
            baseDistance = referenceDetail.distance;
        end
        deltaInsertion = referenceDetail.distance-baseDistance;
        insertionRatio = deltaInsertion/max(abs(baseDistance),eps);
        detour = [];
        for id=newIDs
            pos=find(route==id,1);
            if pos==1
                from=model.depot;
            else
                from=model.customerXYZ(route(pos-1),:);
            end
            if pos==numel(route)
                to=model.homePosition;
            else
                to=model.customerXYZ(route(pos+1),:);
            end
            localNode=find(instance.cache.customerIDs==id,1)+1;
            % Use cached incident legs when available.
            if pos==1
                legIn=norm(from-model.customerXYZ(id,:));
            else
                prevNode=find(instance.cache.customerIDs==route(pos-1),1)+1;
                legIn=instance.cache.legs{prevNode,localNode}.distance;
            end
            if pos==numel(route)
                legOut=norm(model.customerXYZ(id,:)-to);
            else
                nextNode=find(instance.cache.customerIDs==route(pos+1),1)+1;
                legOut=instance.cache.legs{localNode,nextNode}.distance;
            end
            euIn=norm(from-model.customerXYZ(id,:));
            euOut=norm(model.customerXYZ(id,:)-to);
            detour(end+1,1)=(legIn/max(euIn,eps)+legOut/max(euOut,eps))/2; %#ok<AGROW>
        end
        records=referenceDetail.records;
        slack=[];
        for k=1:size(records,1)
            id=records(k,1);
            slack(end+1,1)=model.windows(id,2)-records(k,3); %#ok<AGROW>
        end
        rows{end+1,1}={string(datasets{d}),levelColumn(r),manifest.instance(r), ...
            manifest.scenario_seed(r),numel(newIDs),deltaInsertion,insertionRatio, ...
            mean(detour,'omitnan'),max(detour,[],'omitnan'), ...
            min(slack),mean(slack),sum(slack<30),referenceDetail.distance, ...
            referenceDetail.totalLate,referenceDetail.totalObstacleViolation}; %#ok<AGROW>
    end
end

features=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'dataset','level','instance','scenario_seed','n_add', ...
    'delta_insertion_distance','insertion_ratio','mean_detour_ratio', ...
    'max_detour_ratio','min_slack','mean_slack','critical_count', ...
    'reference_distance','reference_late','reference_obstacle_violation'});
writetable(features,fullfile(resultDir,'event_features.csv'));
save(fullfile(resultDir,'event_features.mat'),'features');
end
