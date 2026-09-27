function results = BuildFormalStatistics
%BUILDFORMALSTATISTICS 生成SPB和动态3D原型的论文统计汇总
root=fileparts(fileparts(mfilename('fullpath'))); setup;
spbFile=fullfile(root,'results','all_spb_mechanism_screen_summary.csv');
t=readtable(spbFile,'TextType','string'); methods=["LateOnly","LateTo2Opt","LateToVND"];
files=unique(t.file,'stable'); gapMatrix=nan(numel(files),numel(methods)); feasibleMatrix=nan(numel(files),numel(methods));
for i=1:numel(files)
    for m=1:numel(methods)
        x=t(t.file==files(i)&t.method==methods(m),:);
        feasibleMatrix(i,m)=mean(logical(x.is_feasible));
        g=double(x.gap_to_bks); g=g(isfinite(g)); if ~isempty(g), gapMatrix(i,m)=median(g); end
    end
end
qualityMask=all(isfinite(gapMatrix),2);
rankMatrix=nan(size(gapMatrix));
for i=1:size(gapMatrix,1), rankMatrix(i,:)=TiedRanks(gapMatrix(i,:)); end
pairNames=["LateToVND_vs_LateOnly","LateToVND_vs_LateTo2Opt"];
pairRows=cell(0,1);
for p=1:2
    if p==1, a=gapMatrix(:,3); b=gapMatrix(:,1); else, a=gapMatrix(:,3); b=gapMatrix(:,2); end
    mask=isfinite(a)&isfinite(b); d=a(mask)-b(mask); [w,tie,l]=WinTieLoss(d);
    [pValue,statValue]=SafeSignrank(d);
    pairRows{end+1,1}={pairNames(p),sum(mask),w,tie,l,median(d),pValue,statValue}; %#ok<AGROW>
end
pairTable=cell2table(vertcat(pairRows{:}),'VariableNames',{'comparison','nPairs','wins','ties','losses','medianDelta','pValue','statistic'});
pairTable.holmP=HolmAdjust(pairTable.pValue);
% Friedman statistic over complete quality-evaluable rows.
complete=all(isfinite(gapMatrix),2); completeGaps=gapMatrix(complete,:); completeRanks=rankMatrix(complete,:);
[friedmanStat,friedmanP]=FriedmanFallback(completeRanks);
rankSummary=table(methods',mean(completeRanks,1)',median(completeRanks,1)', ...
    sum(completeRanks==1,1)', 'VariableNames',{'method','meanRank','medianRank','nRank1'});
rankMeta=table(sum(complete),sum(~complete),friedmanStat,friedmanP, ...
    'VariableNames',{'qualityCompleteInstances','incompleteInstances','friedmanStatistic','friedmanP'});
spb=table(files,feasibleMatrix(:,1),feasibleMatrix(:,2),feasibleMatrix(:,3), ...
    gapMatrix(:,1),gapMatrix(:,2),gapMatrix(:,3),rankMatrix(:,1),rankMatrix(:,2),rankMatrix(:,3), ...
    'VariableNames',{'file','feasibleRate_LateOnly','feasibleRate_LateTo2Opt','feasibleRate_LateToVND', ...
    'medianGap_LateOnly','medianGap_LateTo2Opt','medianGap_LateToVND', ...
    'rank_LateOnly','rank_LateTo2Opt','rank_LateToVND'});
results.spb=spb; results.pairwise=pairTable; results.rankSummary=rankSummary; results.rankMeta=rankMeta;
outDir=fullfile(root,'results');
writetable(spb,fullfile(outDir,'formal_spb_instance_statistics.csv'));
writetable(pairTable,fullfile(outDir,'formal_spb_pairwise_tests.csv'));
writetable(rankSummary,fullfile(outDir,'formal_spb_rank_summary.csv'));
writetable(rankMeta,fullfile(outDir,'formal_spb_rank_meta.csv'));
% Dynamic summary if available.
dynFile=fullfile(outDir,'dynamic_fbcmps0_summary.csv');
if isfile(dynFile)
    d=readtable(dynFile,'TextType','string'); ds=unique(d.strategy,'stable'); cps=unique(d.checkpoint_fe); dr=cell(0,1);
    for c=1:numel(cps)
        for m=1:numel(ds)
            x=d(d.checkpoint_fe==cps(c)&d.strategy==ds(m),:); g=double(x.gap_to_reference); g=g(isfinite(g));
            dr{end+1,1}={ds(m),cps(c),height(x),mean(logical(x.is_feasible)),MedianOrNaN(double(x.total_late)), ...
                MedianOrNaN(g),mean(logical(x.overtake_repair))}; %#ok<AGROW>
        end
    end
    dynamic=cell2table(vertcat(dr{:}),'VariableNames',{'strategy','checkpointFE','nRuns','feasibleRate','medianLate','medianGap','overtakeRepairRate'});
    results.dynamic=dynamic; writetable(dynamic,fullfile(outDir,'formal_dynamic_uav_statistics.csv'));
end
save(fullfile(outDir,'formal_statistics_result.mat'),'results');
fprintf('Formal statistics generated.\n');
end

function ranks=TiedRanks(values)
[sorted,order]=sort(values,'ascend'); ranks=zeros(size(values)); pos=1;
while pos<=numel(values)
    endPos=pos; while endPos<numel(values)&&abs(sorted(endPos+1)-sorted(pos))<=1e-12,endPos=endPos+1;end
    ranks(order(pos:endPos))=(pos+endPos)/2; pos=endPos+1;
end
end
function [w,tie,l]=WinTieLoss(d)
w=sum(d< -1e-12); tie=sum(abs(d)<=1e-12); l=sum(d>1e-12);
end
function [pValue,statValue]=SafeSignrank(d)
try
    [pValue,~,stats]=signrank(d); statValue=stats.signedrank;
catch
    statValue=NaN; pValue=NaN;
end
end
function pAdj=HolmAdjust(p)
[sorted,order]=sort(p); pAdj=nan(size(p)); m=numel(p);
for i=1:m, pAdj(order(i))=min(1,(m-i+1)*sorted(i)); end
for i=m-1:-1:1, pAdj(order(i))=max(pAdj(order(i)),pAdj(order(i+1))); end
end
function value=MedianOrNaN(x)
x=x(isfinite(x)); if isempty(x),value=NaN;else,value=median(x);end
end
function [stat,p]=FriedmanFallback(ranks)
n=size(ranks,1); k=size(ranks,2); R=sum(ranks,1);
stat=12/(n*k*(k+1))*sum(R.^2)-3*n*(k+1);
try, p=1-chi2cdf(stat,k-1); catch, p=NaN; end
end


