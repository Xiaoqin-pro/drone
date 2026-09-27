function results = RunPhaseTransitionBudgetScreen
%RUNPHASETRANSITIONBUDGETSCREEN 只用LateOnly定位phase-transition的有效预算区域
%   不比较Phase-II算法，只记录Phase-I feasibility recovery指标。

root=fileparts(fileparts(mfilename('fullpath'))); setup;
base=fullfile(root,'data','phase_transition_extracted');
scales=[21 31]; betas=[0.70 0.80 0.90 1.00]; realizationIDs=1:5;
budgetsFactor=[100 200 500]; seeds=1:3; rows=cell(0,1);
for scale=scales
    pattern=sprintf('ins_n_%d_alph_1.00_beta_*.txt',scale);
    files=dir(fullfile(base,sprintf('n%d',scale),'**',pattern));
    for beta=betas
        selected=files([]);
        for k=1:numel(files)
            name=string(files(k).name);
            token=regexp(char(name),sprintf('ins_n_%d_alph_1\\.00_beta_([0-9.]+)_([0-9]+)\\.txt',scale),'tokens','once');
            if isempty(token), continue; end
            fileBeta=str2double(token{1}); fileID=str2double(token{2});
            if abs(fileBeta-beta)<1e-9 && ismember(fileID,realizationIDs)
                selected(end+1)=files(k); %#ok<AGROW>
            end
        end
        for f=1:numel(selected)
            fileName=string(selected(f).name);
            instance=ReadTSPTWInstance(fullfile(selected(f).folder,selected(f).name));
            token=regexp(char(fileName),sprintf('ins_n_%d_alph_1\\.00_beta_([0-9.]+)_([0-9]+)\\.txt',scale),'tokens','once');
            realization=str2double(token{2});
            for budgetFactor=budgetsFactor
                budget=budgetFactor*instance.nCustomers;
                for seedIndex=1:numel(seeds)
                    seed=1020000+scale*10000+round(beta*1000)*10+realization*10+seedIndex;
                    rng(seed,'twister');
                    options=struct('nPop',20,'maxFE',budget,'localSearchMode','late', ...
                        'localSearchFE',max(1,instance.nCustomers-1), ...
                        'restorationFE',max(1,instance.nCustomers-1), ...
                        'latePenalty',1000,'silent',true);
                    [best,~,stats]=RandomKeyTSPTWPSO(instance,options,[]); %#ok<ASGLU>
                    first=stats.firstFeasibleFE; if isinf(first), first=NaN; end
                    rows{end+1,1}={scale,beta,realization,fileName,instance.nCustomers, ...
                        budgetFactor,budget,seed,best.Detail.isFeasible,first, ...
                        best.Detail.totalLate,best.Detail.tourCost}; %#ok<AGROW>
                end
            end
        end
        fprintf('n=%d beta=%.2f realizations=%d completed.\n',scale,beta,numel(selected));
    end
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'benchmark_nodes','beta','realization','file','nCustomers','budget_factor', ...
    'budget_fe','seed','is_feasible','first_feasible_fe','total_late','final_cost'});
results.summary=summary; results.budgetsFactor=budgetsFactor; results.betas=betas;
writetable(summary,fullfile(root,'results','phase_transition_budget_screen_summary.csv'));
save(fullfile(root,'results','phase_transition_budget_screen_result.mat'),'results');
fprintf('Phase-transition budget screen completed: %d rows.\n',height(summary));
end
