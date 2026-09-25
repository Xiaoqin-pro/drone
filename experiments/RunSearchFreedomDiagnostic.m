function RunSearchFreedomDiagnostic
%RUNSEARCHFREEDOMDIAGNOSTIC 在有改进空间的实例上诊断搜索自由度与PSO能力
scriptDir=fileparts(mfilename('fullpath')); root=fileparts(scriptDir); resultDir=fullfile(root,'results');
ks=[0 2 4 6 8 12 Inf]; nSeeds=5; maxFE=1000;
cases={'addition_instances','M1',9;'addition_instances','M1',10; ...
    'addition_stress_instances','Near',2;'addition_stress_instances','Near',4; ...
    'addition_stress_instances','Tight',1;'addition_stress_instances','Far',1; ...
    'addition_stress_instances','Far',3;'addition_stress_instances','Far',5};
rows=cell(0,1);
for c=1:size(cases,1)
 dataset=cases{c,1}; level=cases{c,2}; id=cases{c,3};
 if strcmp(dataset,'addition_instances'), mf='addition_manifest.csv'; else, mf='stress_manifest.csv'; end
 manifest=readtable(fullfile(resultDir,dataset,mf),'TextType','string');
 if strcmp(dataset,'addition_instances'), hit=string(manifest.level)==level & manifest.instance==id; else, hit=string(manifest.stress_level)==level & manifest.instance==id; end
 loaded=load(fullfile(resultDir,dataset,manifest.file_name(find(hit,1)))); instance=loaded.instance; model=instance.model; model.cfg.silent=true; cache=instance.cache; active=instance.activeIDs;
 base=CreateModel(); base.windows(:,2)=base.windows(:,2)+300; base.cfg.silent=true; ids=base.customerIDs(:)'; bc=BuildLegCache(base,ids,base.homePosition); rng(860000,'twister'); [prev,~,~]=RoutingPSO(base,bc,100,40,.9,.995,1.7,1.7,[]);
 profile=ComputeImpactProfile(prev.Detail.routeIDs,active,instance.event.customerIDs,model,cache); newIDs=profile.eventIDs; rankOld=profile.rankedOldIDs;
 bks=readtable(fullfile(resultDir,'offline_bks_summary_updated.csv'),'TextType','string'); br=bks.dataset==string(dataset)&bks.level==string(level)&bks.instance==id; bksCost=bks.bks_cost(find(br,1));
 for seedIndex=1:nSeeds
  seed=22000+c*100+seedIndex;
  for ki=1:numel(ks)
   k=ks(ki); if isinf(k), freeIDs=active; else, freeIDs=unique([newIDs,rankOld(1:min(k,numel(rankOld)))],'stable'); end
   rng(seed,'twister'); [sol,~,st]=AffectedRegionPSO_V2(model,cache,profile.repairRoute,freeIDs,33,30,.9,.995,1.7,1.7,maxFE);
   rows{end+1,1}={string(dataset),string(level),id,seedIndex,seed,k,numel(freeIDs), ...
    profile.preprocessFE,st.functionEvaluations,profile.preprocessFE+st.functionEvaluations, ...
    bksCost,sol.Cost,(sol.Cost-bksCost)/max(abs(bksCost),eps), ...
    (profile.repairCost-sol.Cost)/max(abs(profile.repairCost),eps), ...
    st.uniqueRouteRatio,st.routeChangeRate,st.improvementCount,st.lastImprovementFE,sol.Detail.isFeasible}; %#ok<AGROW>
  end
 end
 fprintf('%s %s %d completed.\n',dataset,level,id);
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
 'dataset','level','instance','seed_index','algorithm_seed','k','free_count', ...
 'preprocess_fe','search_fe','total_fe','bks_cost','cost','gap_to_bks', ...
 'improvement_over_repair','unique_route_ratio','route_change_rate', ...
 'improvement_count','last_improvement_fe','is_feasible'});
writetable(summary,fullfile(resultDir,'search_freedom_diagnostic.csv')); save(fullfile(resultDir,'search_freedom_diagnostic.mat'),'summary','ks');
end
