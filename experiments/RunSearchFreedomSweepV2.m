function RunSearchFreedomSweepV2
%RUNSEARCHFREEDOMSWEEPV2 统一partial-key下的搜索自由度扫描
scriptDir=fileparts(mfilename('fullpath')); root=fileparts(scriptDir); resultDir=fullfile(root,'results');
cases={'addition_instances','M4',1;'addition_instances','M4',2;'addition_stress_instances','Near',2;'addition_stress_instances','Near',5;'addition_stress_instances','Tight',1;'addition_stress_instances','Tight',4;'addition_stress_instances','Far',3;'addition_stress_instances','Far',5};
kList=[0 2 4 6 8 12 Inf]; rows=cell(0,1);
for c=1:size(cases,1)
 dataset=cases{c,1}; level=cases{c,2}; id=cases{c,3};
 if strcmp(dataset,'addition_instances'), mf='addition_manifest.csv'; else, mf='stress_manifest.csv'; end
 manifest=readtable(fullfile(resultDir,dataset,mf),'TextType','string');
 if strcmp(dataset,'addition_instances'), hit=string(manifest.level)==level & manifest.instance==id; else, hit=string(manifest.stress_level)==level & manifest.instance==id; end
 loaded=load(fullfile(resultDir,dataset,manifest.file_name(find(hit,1)))); instance=loaded.instance; model=instance.model; model.cfg.silent=true; cache=instance.cache; active=instance.activeIDs;
 base=CreateModel(); base.windows(:,2)=base.windows(:,2)+300; base.cfg.silent=true; ids=base.customerIDs(:)'; bc=BuildLegCache(base,ids,base.homePosition); rng(860000,'twister'); [prev,~,~]=RoutingPSO(base,bc,100,40,.9,.995,1.7,1.7,[]);
 profile=ComputeImpactProfile(prev.Detail.routeIDs,active,instance.event.customerIDs,model,cache); rankOld=profile.rankedOldIDs; newIDs=profile.eventIDs;
 for kk=kList
  if isinf(kk), freeIDs=active; label='Full'; else, freeIDs=unique([newIDs,rankOld(1:min(kk,numel(rankOld)))],'stable'); label=sprintf('k%d',kk); end
  rng(18000+c*100+find(kList==kk,1),'twister'); [sol,~,st]=AffectedRegionPSO_V2(model,cache,profile.repairRoute,freeIDs,33,30,.9,.995,1.7,1.7,1000);
  rows{end+1,1}={string(dataset),string(level),id,string(label),numel(freeIDs),numel(active),st.functionEvaluations,sol.Cost,(sol.Cost-instance.referenceCost)/max(abs(instance.referenceCost),eps),sol.Detail.isFeasible,profile.repairCost,profile.preprocessFE}; %#ok<AGROW>
 end
 fprintf('%s %s %d profile affected=%d/%d\n',dataset,level,id,numel(profile.affectedIDs),numel(active));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{'dataset','level','instance','region','free_count','active_count','actual_fe','fitness','gap_to_witness','is_feasible','repair_incumbent_cost','preprocess_fe'}); writetable(summary,fullfile(resultDir,'search_space_sweep_v2.csv')); save(fullfile(resultDir,'search_space_sweep_v2.mat'),'summary','kList');
end
