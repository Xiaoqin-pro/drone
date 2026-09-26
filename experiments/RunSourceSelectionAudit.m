function results = RunSourceSelectionAudit
%RUNSOURCESELECTIONAUDIT 在相同不可行路线集上比较Global/Late/Propagation source选择

root=fileparts(fileparts(mfilename('fullpath'))); setup;
benchmarkDir=fullfile(root,'data','tsp_tw_benchmark');
manifest=readtable(fullfile(benchmarkDir,'manifest.csv'),'Delimiter',',','TextType','string');
methods=["Global","LateOnly","Propagation"]; nRoutes=20; rows=cell(0,1);
for i=1:height(manifest)
    instance=ReadTSPTWInstance(fullfile(benchmarkDir,manifest.file(i)));
    routeCount=0; attempt=0;
    while routeCount<nRoutes && attempt<5000
        attempt=attempt+1; rng(950000+100*i+attempt,'twister');
        route=instance.customerIDs(randperm(instance.nCustomers));
        [~,detail]=EvaluateTSPTWRoute(route,instance,struct('latePenalty',1000));
        if detail.isFeasible, continue; end
        routeCount=routeCount+1;
        [lateSource,propSource,scores]=SelectSources(route,detail); %#ok<ASGLU>
        for methodIndex=1:numel(methods)
            method=methods(methodIndex); rng(960000+1000*i+100*routeCount+methodIndex,'twister');
            switch method
                case "Global", sourceNode=route(randi(numel(route)));
                case "LateOnly", sourceNode=lateSource;
                case "Propagation", sourceNode=propSource;
            end
            [~,~,bestDetail,stats]=EvaluateSingleSourceRelocate(route,instance, ...
                sourceNode,detail,struct('latePenalty',1000));
            relativeReduction=max(0,(detail.totalLate-bestDetail.totalLate)/ ...
                max(detail.totalLate,eps));
            rows{end+1,1}={manifest.name(i),routeCount,method,sourceNode, ...
                lateSource,propSource,lateSource~=propSource,detail.totalLate, ...
                bestDetail.totalLate,relativeReduction,bestDetail.isFeasible, ...
                stats.functionEvaluations}; %#ok<AGROW>
        end
    end
    fprintf('%s: %d identical infeasible routes audited.\n',manifest.name(i),routeCount);
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'instance','route_index','method','source_node','late_source', ...
    'propagation_source','source_disagreement','late_before','late_after', ...
    'relative_late_reduction','reaches_feasibility','source_fe'});
results.summary=summary; results.methods=methods;
writetable(summary,fullfile(root,'results','source_selection_audit_summary.csv'));
save(fullfile(root,'results','source_selection_audit_result.mat'),'results');
fprintf('Source selection audit completed: %d rows.\n',height(summary));
end

function [lateSource,propSource,scores]=SelectSources(route,detail)
route=route(:)'; n=numel(route); late=zeros(1,n); suffix=zeros(n,1);
for k=size(detail.records,1):-1:1
    suffix(k)=max(0,detail.records(k,4));
    if k<size(detail.records,1), suffix(k)=suffix(k)+suffix(k+1); end
    idx=find(route==detail.records(k,1),1);
    if ~isempty(idx), late(idx)=max(0,detail.records(k,4)); end
end
prop=zeros(1,n);
for k=1:n
    prop(k)=late(k)+0.5*suffix(k)/max(n-k+1,1);
end
[~,lateIndex]=max(late); [~,propIndex]=max(prop);
lateSource=route(lateIndex); propSource=route(propIndex); scores=prop;
end
