function results = RunFeasibleSourceFeatureAudit
%RUNFEASIBLESOURCEFEATUREAUDIT 路线级Feature Audit V1
%   不改算法，只评价简单source特征对真实oracle gain的预测能力。

root=fileparts(fileparts(mfilename('fullpath'))); setup;
sourceFile=fullfile(root,'results','feasible_source_oracle_v2_sources.csv');
routeFile=fullfile(root,'results','feasible_source_oracle_v2_routes.csv');
sources=readtable(sourceFile,'TextType','string'); routes=readtable(routeFile,'TextType','string');
features=["removal_saving","slack","waiting","local_travel"];
rows=cell(0,1); routeRows=cell(0,1);
keys=unique(sources(:,{'instance','route_index'}),'rows');
for r=1:height(keys)
    mask=sources.instance==keys.instance(r) & sources.route_index==keys.route_index(r) & sources.actionable;
    data=sources(mask,:); if height(data)==0, continue; end
    routeMask=routes.instance==keys.instance(r) & routes.route_index==keys.route_index(r);
    routeSource=string(routes.route_source(routeMask));
    for f=1:numel(features)
        featureName=features(f); values=double(data.(featureName)); gains=double(data.source_gain);
        rho=SpearmanCorrelation(values,gains);
        for direction=["max","min"]
            if direction=="max", [~,selectedIndex]=max(values); else, [~,selectedIndex]=min(values); end
            selectedGain=gains(selectedIndex); oracleGain=max(gains);
            [~,oracleIndex]=max(gains); [~,rankOrder]=sort(gains,'descend'); rank=find(rankOrder==selectedIndex,1);
            capture=selectedGain/max(oracleGain,eps);
            routeRows{end+1,1}={keys.instance(r),keys.route_index(r),routeSource, ...
                featureName,direction,rho,rank,(rank-1)/max(height(data)-1,1), ...
                selectedGain,oracleGain,capture,selectedGain>1e-10}; %#ok<AGROW>
        end
    end
end
routeAudit=cell2table(vertcat(routeRows{:}),'VariableNames',{ ...
    'instance','route_index','route_source','feature','direction','spearman_rho', ...
    'oracle_rank','normalized_rank','selected_gain','oracle_gain','gain_capture', ...
    'selected_source_improves'});
combinations=unique(routeAudit(:,{'instance','feature','direction'}),'rows');
for i=1:height(combinations)
    m=routeAudit.instance==combinations.instance(i) & ...
        routeAudit.feature==combinations.feature(i) & ...
        routeAudit.direction==combinations.direction(i);
    x=routeAudit(m,:); rhos=double(x.spearman_rho);
    rows{end+1,1}={combinations.instance(i),combinations.feature(i), ...
        combinations.direction(i),height(x),median(rhos,'omitnan'),IQR(rhos), ...
        mean(rhos>0,'omitnan'),mean(double(x.oracle_rank)==1), ...
        mean(double(x.oracle_rank)<=3),median(double(x.normalized_rank),'omitnan'), ...
        median(double(x.gain_capture),'omitnan'), ...
        mean(logical(x.selected_source_improves))}; %#ok<AGROW>
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'instance','feature','direction','route_count','median_rho','iqr_rho', ...
    'positive_rho_fraction','oracle_hit_rate','top3_rate','median_normalized_rank', ...
    'median_gain_capture','selected_improve_rate'});
results.summary=summary; results.routeAudit=routeAudit;
writetable(summary,fullfile(root,'results','feasible_source_feature_audit_summary.csv'));
writetable(routeAudit,fullfile(root,'results','feasible_source_feature_audit_routes.csv'));
save(fullfile(root,'results','feasible_source_feature_audit_result.mat'),'results');
fprintf('Feature audit completed: %d route-feature rows.\n',height(routeAudit));
end

function rho=SpearmanCorrelation(x,y)
x=x(:); y=y(:); valid=isfinite(x)&isfinite(y); x=x(valid); y=y(valid);
if numel(x)<3 || std(x)==0 || std(y)==0, rho=NaN; return; end
rx=AverageRanks(x); ry=AverageRanks(y); rho=corr(rx,ry);
end

function ranks=AverageRanks(x)
[sorted,order]=sort(x); ranks=zeros(size(x)); k=1;
while k<=numel(x)
    j=k;
    while j<numel(x) && sorted(j+1)==sorted(k), j=j+1; end
    ranks(order(k:j))=(k+j)/2; k=j+1;
end
end

function value=IQR(x)
x=x(isfinite(x)); if isempty(x), value=NaN; else, value=quantile(x,0.75)-quantile(x,0.25); end
end
