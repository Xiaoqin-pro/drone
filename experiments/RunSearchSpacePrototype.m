function RunSearchSpacePrototype
%RUNSEARCHSPACEPROTOTYPE 比较New-only/Affected-V2/Expanded/Full搜索空间
scriptDir=fileparts(mfilename('fullpath'));
root=fileparts(scriptDir);
resultDir=fullfile(root,'results');
cases={
    'addition_instances','M4',1;
    'addition_instances','M4',2;
    'addition_stress_instances','Near',2;
    'addition_stress_instances','Near',5;
    'addition_stress_instances','Tight',1;
    'addition_stress_instances','Tight',4;
    'addition_stress_instances','Far',3;
    'addition_stress_instances','Far',5};
regions={'NewOnly','AffectedV2','Expanded','Full'};
rows=cell(0,1);
for c=1:size(cases,1)
    dataset=cases{c,1}; level=cases{c,2}; id=cases{c,3};
    if strcmp(dataset,'addition_instances'), mf='addition_manifest.csv'; else, mf='stress_manifest.csv'; end
    manifest=readtable(fullfile(resultDir,dataset,mf),'TextType','string');
    if strcmp(dataset,'addition_instances')
        hit=string(manifest.level)==level & manifest.instance==id;
    else
        hit=string(manifest.stress_level)==level & manifest.instance==id;
    end
    fileName=manifest.file_name(find(hit,1));
    instance=load(fullfile(resultDir,dataset,fileName)); instance=instance.instance;
    model=instance.model; model.cfg.silent=true; cache=instance.cache; activeIDs=instance.activeIDs;
    base=CreateModel(); base.windows(:,2)=base.windows(:,2)+300; base.cfg.silent=true;
    baseIDs=base.customerIDs(:)'; baseCache=BuildLegCache(base,baseIDs,base.homePosition);
    rng(860000,'twister');
    [prev,~,~]=RoutingPSO(base,baseCache,100,40,0.90,0.995,1.7,1.7,[]);
    [affected,info]=DetectAffectedRegion(prev.Detail.routeIDs,activeIDs, ...
        instance.event.customerIDs,model,cache);
    newOnly=intersect(instance.event.customerIDs,activeIDs,'stable');
    expanded=ExpandRegion(affected,prev.Detail.routeIDs,activeIDs,2);
    sets={newOnly,affected,expanded,activeIDs};
    for s=1:numel(regions)
        rng(15000+c*100+s,'twister');
        [sol,~,stats]=AffectedRegionPSO(model,cache,prev.Detail.routeIDs, ...
            sets{s},33,30,0.90,0.995,1.7,1.7,false,1000);
        rows{end+1,1}={string(dataset),string(level),id,string(regions{s}), ...
            numel(sets{s}),numel(sets{s})/numel(activeIDs),stats.functionEvaluations, ...
            sol.Cost,(sol.Cost-instance.referenceCost)/max(abs(instance.referenceCost),eps), ...
            sol.Detail.isFeasible,info.repairCost,info.affectedCount}; %#ok<AGROW>
    end
    fprintf('%s %s %d: new=%d affected=%d expanded=%d full=%d\n', ...
        dataset,level,id,numel(newOnly),numel(affected),numel(expanded),numel(activeIDs));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'dataset','level','instance','region','region_count','region_ratio', ...
    'actual_fe','fitness','gap_to_witness','is_feasible','direct_repair_cost', ...
    'affected_v2_count'});
writetable(summary,fullfile(resultDir,'search_space_prototype.csv'));
save(fullfile(resultDir,'search_space_prototype.mat'),'summary','cases','regions');
end

function expanded=ExpandRegion(seed,route,activeIDs,radius)
expanded=seed(:)';
for id=seed(:)'
    pos=find(route==id,1);
    if isempty(pos), continue; end
    expanded=union(expanded,route(max(1,pos-radius):min(numel(route),pos+radius)),'stable');
end
expanded=intersect(expanded,activeIDs,'stable');
end
