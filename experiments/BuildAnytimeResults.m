function BuildAnytimeResults
%BUILDANYTIMERESULTS 汇总8个Restart长预算与Repair/Warm检查点
scriptDir=fileparts(mfilename('fullpath'));
root=fileparts(scriptDir);
resultDir=fullfile(root,'results');
long=readtable(fullfile(resultDir,'restart_long_summary.csv'),'TextType','string');
rows=cell(0,1);
for r=1:height(long)
    dataset=long.dataset(r); level=long.level(r); instance=long.instance(r); fe=long.checkpoint_fe(r);
    if dataset=="addition_instances"
        file=fullfile(resultDir,'addition_paired_bks_gap.csv');
        data=readtable(file,'TextType','string');
        hit=string(data.level)==level & data.instance==instance & data.budget==fe;
    else
        file=fullfile(resultDir,'stress_addition_bks_gap.csv');
        data=readtable(file,'TextType','string');
        hit=string(data.stress_level)==level & data.instance==instance & data.budget==fe;
    end
    if any(hit)
        rr=data(hit,:);
        for k=1:height(rr)
            rows{end+1,1}={dataset,level,instance,fe, ...
                string(rr.strategy(k)),rr.fitness(k),rr.gap_to_bks(k), ...
                rr.total_fe(k),rr.is_feasible(k)}; %#ok<AGROW>
        end
    end
    rows{end+1,1}={dataset,level,instance,fe,"Restart", ...
        long.distance(r),long.gap_to_bks(r),long.actual_fe(r), ...
        long.is_feasible(r)}; %#ok<AGROW>
end
anytime=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'dataset','level','instance','checkpoint_fe','strategy', ...
    'fitness','gap_to_bks','actual_fe','is_feasible'});
writetable(anytime,fullfile(resultDir,'anytime_comparison_summary.csv'));
save(fullfile(resultDir,'anytime_comparison_result.mat'),'anytime');
end
