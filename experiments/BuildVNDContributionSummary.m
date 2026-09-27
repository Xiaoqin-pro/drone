function results = BuildVNDContributionSummary
%BUILDVNDCONTRIBUTIONSUMMARY 汇总已有VND/公平fork机制证据，不重新运行算法
root=fileparts(fileparts(mfilename('fullpath'))); setup;
rows=cell(0,1);
% 1) 完整PSO decision：VND各邻域accept统计
file=fullfile(root,'results','boundary_vnd_decision_100n_summary.csv');
if isfile(file)
    t=readtable(file,'TextType','string');
    x=t(t.method=="LateToVND",:);
    rows{end+1,1}={"integrated","LateToVND",height(x), ...
        mean(x.vnd_2opt_accept>0),mean(x.vnd_swap_accept>0), ...
        mean(x.vnd_relocate_accept>0),mean((x.vnd_2opt_accept>0)+ ...
        (x.vnd_swap_accept>0)+(x.vnd_relocate_accept>0)>=2), ...
        mean(x.vnd_2opt_accept+x.vnd_swap_accept+x.vnd_relocate_accept), ...
        NaN,NaN,NaN}; %#ok<AGROW>
end
% 2) Fair fork V2 paired state wins: VND and Competitive against Repeated2Opt
file=fullfile(root,'results','competitive_neighborhood_fork_v2.csv');
if isfile(file)
    t=readtable(file,'TextType','string');
    keys=unique(t(:,{'file','seed'}),'rows');
    for method=["BudgetedVND","Competitive"]
        wins=0; ties=0; losses=0; deltas=[];
        for k=1:height(keys)
            m=t.file==keys.file(k)&t.seed==keys.seed(k);
            a=t.post_gain(m & t.method==method); b=t.post_gain(m & t.method=="Repeated2Opt");
            if isempty(a)||isempty(b), continue; end
            d=a(1)-b(1); deltas(end+1)=d; %#ok<AGROW>
            if d>1e-10, wins=wins+1; elseif d<-1e-10, losses=losses+1; else, ties=ties+1; end
        end
        rows{end+1,1}={"fork-v2",method,numel(deltas),NaN,NaN,NaN,NaN, ...
            NaN,wins,ties,losses}; %#ok<AGROW>
    end
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'source','method','n_runs','twoOpt_participation','swap_participation', ...
    'relocate_participation','multi_operator_run_fraction','mean_accepted_moves', ...
    'wins_vs_repeated2opt','ties_vs_repeated2opt','losses_vs_repeated2opt'});
results.summary=summary;
writetable(summary,fullfile(root,'results','vnd_contribution_summary.csv'));
save(fullfile(root,'results','vnd_contribution_summary.mat'),'results');
fprintf('VND contribution summary written: %d rows.\n',height(summary));
end
