function results = BuildAllSPBPaperSummary
%BUILDALLSPBPAPERSUMMARY 生成30个SPB实例的instance-balanced论文汇总
%   不重新运行算法；对每个实例先聚合seed，再进行实例层比较。

root=fileparts(fileparts(mfilename('fullpath'))); setup;
inputFile=fullfile(root,'results','all_spb_mechanism_screen_summary.csv');
t=readtable(inputFile,'TextType','string');
methods=["LateOnly","LateTo2Opt","LateToVND"];
files=unique(t.file,'stable'); rows=cell(0,1);

for i=1:numel(files)
    perMethod=struct();
    for m=1:numel(methods)
        x=t(t.file==files(i) & t.method==methods(m),:);
        feasible=logical(x.is_feasible);
        gaps=double(x.gap_to_bks); gaps=gaps(isfinite(gaps));
        phaseGain=double(x.phase_gain); phaseGain=phaseGain(isfinite(phaseGain));
        phaseEff=double(x.phase_gain_per_fe); phaseEff=phaseEff(isfinite(phaseEff));
        first=double(x.first_feasible_fe); first=first(isfinite(first));
        perMethod(m).feasibleRate=mean(feasible);
        perMethod(m).medianGap=MedianOrNaN(gaps);
        perMethod(m).iqrGap=IQROrNaN(gaps);
        perMethod(m).medianPhaseGain=MedianOrNaN(phaseGain);
        perMethod(m).medianPhaseEff=MedianOrNaN(phaseEff);
        perMethod(m).medianFirstFE=MedianOrNaN(first);
        perMethod(m).nFeasible=sum(feasible);
    end
    % Gap comparisons only use instances where both methods have feasible runs.
    pair12=CompareValues(perMethod(3).medianGap,perMethod(1).medianGap);
    pair13=CompareValues(perMethod(3).medianGap,perMethod(2).medianGap);
    rows{end+1,1}={files(i),x.nCustomers(1), ...
        perMethod(1).feasibleRate,perMethod(2).feasibleRate,perMethod(3).feasibleRate, ...
        perMethod(1).medianGap,perMethod(2).medianGap,perMethod(3).medianGap, ...
        perMethod(1).iqrGap,perMethod(2).iqrGap,perMethod(3).iqrGap, ...
        perMethod(1).medianPhaseGain,perMethod(2).medianPhaseGain,perMethod(3).medianPhaseGain, ...
        perMethod(1).medianFirstFE,perMethod(2).medianFirstFE,perMethod(3).medianFirstFE, ...
        pair12,pair13}; %#ok<AGROW>
end

perInstance=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','nCustomers','feasibleRate_LateOnly','feasibleRate_LateTo2Opt', ...
    'feasibleRate_LateToVND','medianGap_LateOnly','medianGap_LateTo2Opt', ...
    'medianGap_LateToVND','iqrGap_LateOnly','iqrGap_LateTo2Opt','iqrGap_LateToVND', ...
    'medianPhaseGain_LateOnly','medianPhaseGain_LateTo2Opt','medianPhaseGain_LateToVND', ...
    'medianFirstFE_LateOnly','medianFirstFE_LateTo2Opt','medianFirstFE_LateToVND', ...
    'vndVsLateOnly','vndVsLateTo2Opt'});

% Instance-level ranks: feasible rate first, then median gap among feasible methods.
rankRows=cell(0,1); rankMatrix=nan(height(perInstance),numel(methods));
for i=1:height(perInstance)
    feasibleRates=[perInstance.feasibleRate_LateOnly(i), ...
        perInstance.feasibleRate_LateTo2Opt(i),perInstance.feasibleRate_LateToVND(i)];
    gaps=[perInstance.medianGap_LateOnly(i),perInstance.medianGap_LateTo2Opt(i),perInstance.medianGap_LateToVND(i)];
    % Primary: higher feasible rate. Secondary: lower feasible-run median gap.
    score=[-feasibleRates(:),FillNaN(gaps(:),inf)];
    [~,order]=sortrows(score,[1 2]);
    ranks=zeros(1,3); ranks(order)=1:3; rankMatrix(i,:)=ranks;
    rankRows{end+1,1}={perInstance.file(i),ranks(1),ranks(2),ranks(3), ...
        order(1)==3,order(1)==2,order(1)==1}; %#ok<AGROW>
end
rankTable=cell2table(vertcat(rankRows{:}),'VariableNames',{ ...
    'file','rank_LateOnly','rank_LateTo2Opt','rank_LateToVND', ...
    'bestIsLateToVND','bestIsLateTo2Opt','bestIsLateOnly'});

summary=table(methods',mean(rankMatrix,1)',median(rankMatrix,1)', ...
    sum(rankMatrix==1,1)',sum(rankMatrix==2,1)',sum(rankMatrix==3,1)', ...
    'VariableNames',{'method','averageRank','medianRank','nRank1','nRank2','nRank3'});
% Paired instance-level W/T/L for median gaps.
[wLate,tLate,lLate]=WinTieLoss(perInstance.medianGap_LateToVND,perInstance.medianGap_LateOnly);
[w2,t2,l2]=WinTieLoss(perInstance.medianGap_LateToVND,perInstance.medianGap_LateTo2Opt);
pairSummary=table(["LateToVND_vs_LateOnly";"LateToVND_vs_LateTo2Opt"], ...
    [wLate;w2],[tLate;t2],[lLate;l2], ...
    'VariableNames',{'comparison','wins','ties','losses'});

results.perInstance=perInstance; results.rankTable=rankTable; results.summary=summary; results.pairSummary=pairSummary;
outDir=fullfile(root,'results');
writetable(perInstance,fullfile(outDir,'all_spb_instance_balanced_summary.csv'));
writetable(rankTable,fullfile(outDir,'all_spb_instance_ranks.csv'));
writetable(summary,fullfile(outDir,'all_spb_method_rank_summary.csv'));
writetable(pairSummary,fullfile(outDir,'all_spb_instance_paired_wtl.csv'));
save(fullfile(outDir,'all_spb_paper_summary.mat'),'results');
fprintf('Paper summary generated for %d SPB instances.\n',height(perInstance));
end

function value=MedianOrNaN(x)
if isempty(x), value=NaN; else, value=median(x); end
end
function value=IQROrNaN(x)
if isempty(x), value=NaN; else, value=quantile(x,0.75)-quantile(x,0.25); end
end
function x=FillNaN(x,value)
x(~isfinite(x))=value;
end
function result=CompareValues(a,b)
if ~isfinite(a)||~isfinite(b), result="NA";
elseif a<b-1e-12, result="VND";
elseif a>b+1e-12, result="Other";
else, result="Tie"; end
end
function [w,t,l]=WinTieLoss(a,b)
valid=isfinite(a)&isfinite(b); d=a(valid)-b(valid);
w=sum(d< -1e-12); t=sum(abs(d)<=1e-12); l=sum(d>1e-12);
end
