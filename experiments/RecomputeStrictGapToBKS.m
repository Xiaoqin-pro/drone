function RecomputeStrictGapToBKS
%RECOMPUTESTRICTGAPTOBKS 将Strict Anytime结果统一换算为BKS Gap
scriptDir=fileparts(mfilename('fullpath'));
root=fileparts(scriptDir);
resultDir=fullfile(root,'results');
strict=readtable(fullfile(resultDir,'strict_anytime_summary.csv'),'TextType','string');
bks=readtable(fullfile(resultDir,'offline_bks_summary_updated.csv'),'TextType','string');
strict.bks_cost=nan(height(strict),1);
strict.gap_to_bks=nan(height(strict),1);
for r=1:height(strict)
    hit=bks.dataset==strict.dataset(r) & bks.level==strict.level(r) ...
        & bks.instance==strict.instance(r);
    if any(hit)
        value=bks.bks_cost(find(hit,1));
        strict.bks_cost(r)=value;
        strict.gap_to_bks(r)=(strict.cost(r)-value)/max(abs(value),eps);
    end
end
writetable(strict,fullfile(resultDir,'strict_anytime_bks_gap.csv'));
end
