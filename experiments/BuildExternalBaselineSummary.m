function results = BuildExternalBaselineSummary
%BUILDEXTERNALBASELINESUMMARY 汇总FB-CMPSO与DD-LNS外部协议结果
%   内部FB结果来自30 SPB×10 seeds；DD-LNS来自published protocol×5 seeds。
%   两者不合并到同一FE排名，仅做native-protocol竞争性对照。

root=fileparts(fileparts(mfilename('fullpath'))); setup;
fb=readtable(fullfile(root,'results','all_spb_mechanism_screen_summary.csv'),'TextType','string');
dd=readtable(fullfile(root,'results','ddlns_formal_spb_summary.csv'),'TextType','string');
files=unique(fb.file,'stable'); rows=cell(0,1);
for i=1:numel(files)
    f=files(i); a=fb(fb.file==f & fb.method=="LateToVND",:); b=dd(dd.file==f & dd.status=="solved",:);
    fbGap=double(a.gap_to_bks); fbGap=fbGap(isfinite(fbGap)); ddGap=double(b.gap_to_bks); ddGap=ddGap(isfinite(ddGap));
    fbHit=sum(abs(fbGap)<=0.01); ddHit=sum(abs(ddGap)<=0.01);
    rows{end+1,1}={f,a.nCustomers(1),height(a),mean(logical(a.is_feasible)), ...
        MedianOrNaN(fbGap),fbHit,height(b),mean(b.status=="solved"),MedianOrNaN(ddGap),ddHit, ...
        MedianOrNaN(double(b.time_to_best)),MedianOrNaN(double(b.wall_time)), ...
        CompareValues(MedianOrNaN(fbGap),MedianOrNaN(ddGap))}; %#ok<AGROW>
end
perInstance=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','nCustomers','fb_runs','fb_feasible_rate','fb_median_gap','fb_bks_hits', ...
    'ddlns_runs','ddlns_solved_rate','ddlns_median_gap','ddlns_bks_hits', ...
    'ddlns_median_time_to_best','ddlns_median_wall_time','fb_vs_ddlns_gap'});
mask=isfinite(perInstance.fb_median_gap)&isfinite(perInstance.ddlns_median_gap);
[w,t,l]=WinTieLoss(perInstance.fb_median_gap(mask),perInstance.ddlns_median_gap(mask));
summary=table(sum(mask),w,t,l,mean(perInstance.fb_feasible_rate),mean(perInstance.ddlns_solved_rate), ...
    sum(perInstance.fb_bks_hits),sum(perInstance.ddlns_bks_hits), ...
    'VariableNames',{'commonQualityInstances','fbWins','ties','fbLosses', ...
    'meanFBFeasibleRate','meanDDLNSolvedRate','fbBksHits','ddlnsBksHits'});
results.perInstance=perInstance; results.summary=summary;
outDir=fullfile(root,'results');
writetable(perInstance,fullfile(outDir,'external_spb_instance_comparison.csv'));
writetable(summary,fullfile(outDir,'external_spb_summary.csv'));
save(fullfile(outDir,'external_spb_comparison_result.mat'),'results');
fprintf('External baseline summary generated: %d instances.\n',height(perInstance));
end
function v=MedianOrNaN(x)
x=x(isfinite(x)); if isempty(x),v=NaN;else,v=median(x);end
end
function [w,t,l]=WinTieLoss(a,b)
d=a-b; w=sum(d< -1e-12);t=sum(abs(d)<=1e-12);l=sum(d>1e-12);
end
function x=CompareValues(a,b)
if ~isfinite(a)||~isfinite(b),x="NA";elseif a<b-1e-12,x="FB-CMPSO";elseif a>b+1e-12,x="DD-LNS";else,x="Tie";end
end
