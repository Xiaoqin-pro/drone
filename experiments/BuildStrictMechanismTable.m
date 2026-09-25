function BuildStrictMechanismTable
%BUILDSTRICTMECHANISMTABLE 合并Strict Anytime和事件影响特征
scriptDir=fileparts(mfilename('fullpath'));
root=fileparts(scriptDir);
resultDir=fullfile(root,'results');
strict=readtable(fullfile(resultDir,'strict_anytime_bks_gap.csv'),'TextType','string');
features=readtable(fullfile(resultDir,'event_features.csv'),'TextType','string');
keys=unique(strict(:,{'dataset','level','instance','checkpoint_fe'}));
rows=cell(0,1);
for r=1:height(keys)
    key=keys(r,:);
    hit=strict.dataset==key.dataset & strict.level==key.level ...
        & strict.instance==key.instance & strict.checkpoint_fe==key.checkpoint_fe;
    block=strict(hit,:);
    get=@(name) GetStrategyValue(block,name);
    fh=features.dataset==key.dataset & features.level==key.level ...
        & features.instance==key.instance;
    ft=features(find(fh,1),:);
    repairGap=get('Repair'); warmGap=get('WarmStart'); coldGap=get('Restart');
    repairFeas=getFeasible(block,'Repair'); warmFeas=getFeasible(block,'WarmStart'); coldFeas=getFeasible(block,'Restart');
    rows{end+1,1}={key.dataset,key.level,key.instance,key.checkpoint_fe, ...
        ft.n_add,ft.insertion_ratio,ft.slack_loss,ft.min_slack_after, ...
        ft.critical_count_after,ft.mean_detour_ratio,repairGap,warmGap, ...
        coldGap,repairFeas,warmFeas,coldFeas,repairGap-warmGap, ...
        warmGap-coldGap,BestStrategy(repairGap,warmGap,coldGap, ...
        repairFeas,warmFeas,coldFeas)}; %#ok<AGROW>
end
mechanism=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'dataset','level','instance','checkpoint_fe','n_add', ...
    'insertion_ratio','slack_loss','min_slack_after','critical_count', ...
    'mean_detour_ratio','repair_gap','warm_gap','restart_gap', ...
    'repair_feasible','warm_feasible','restart_feasible', ...
    'warm_gain_vs_repair','restart_gain_vs_warm','best_strategy'});
writetable(mechanism,fullfile(resultDir,'strict_mechanism_table.csv'));
save(fullfile(resultDir,'strict_mechanism_table.mat'),'mechanism');
end

function value=GetStrategyValue(block,name)
hit=string(block.strategy)==name;
if any(hit), value=block.gap_to_bks(find(hit,1)); else, value=NaN; end
end
function value=getFeasible(block,name)
hit=string(block.strategy)==name;
if any(hit), value=block.is_feasible(find(hit,1)); else, value=false; end
end
function name=BestStrategy(a,b,c,fa,fb,fc)
values=[a b c]; values(~[fa fb fc])=inf;
[~,i]=min(values); names={'Repair','WarmStart','Restart'};
if isinf(values(i)), name="None"; else, name=string(names{i}); end
end
