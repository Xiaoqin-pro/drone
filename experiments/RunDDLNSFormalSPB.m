function results = RunDDLNSFormalSPB
%RUND DLNSFORMALSPB 正式外部DD-LNS对比：30 SPB × 5 seeds × 5秒

root=fileparts(fileparts(mfilename('fullpath'))); setup;
exe=fullfile(root,'external_baselines','ijcai_22_DDLNS-main','source-code','solver-ddlns','target','release','examples','tsptw.exe');
bench=fullfile(root,'external_baselines','ijcai_22_DDLNS-main','benchmarks','tsptw');
initFile=fullfile(bench,'initial_solutions','SolomonPotvinBengio.txt');
localDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
reference=ReadTSPTWReferenceSolutions(fullfile(root,'data','tsp_tw_raw','best_known', ...
    'SolomonPotvinBengio-best-known-traveltime.txt'));
files=sort(string({dir(fullfile(localDir,'*.txt')).name})); seeds=1:5; rows=cell(0,1);
lines=readlines(initFile);
for k=1:numel(files)
    name=files(k); line=lines(startsWith(strtrim(lines),name));
    if isempty(line), warning('missing initial solution: %s',name); continue; end
    token=regexp(char(line(1)),'^\S+\s+[-+]?\d*\.?\d+\s+\d+\s+(.*)$','tokens','once');
    if isempty(token), warning('cannot parse initial solution: %s',name); continue; end
    initialRouteText=strtrim(token{1}); externalFile=FindExternalFile(bench,name);
    localFile=fullfile(localDir,name); instance=ReadTSPTWInstance(localFile);
    refIndex=find(strcmpi(string({reference.file}),name),1); if isempty(refIndex), refCost=NaN; else, refCost=reference(refIndex).cost; end
    for s=1:numel(seeds)
        command=sprintf('"%s" solve -f "%s" -w 100 -t 5 -s %d --solution "%s" -H', ...
            exe,externalFile,20211105+s,initialRouteText);
        timer=tic; [status,output]=system(command); wall=toc(timer);
        token=regexp(output,'\|\s*lns\s*\|\s*[^|]+\|\s*([0-9.]+)\s*\|[^|]*\|\s*([0-9.]+|N\.A\.)\s*\|\s*([^|]+)\|\s*(.*)','tokens','once');
        routeValues=[]; externalCost=NaN; bestTime=NaN; routeText=""; state="process_failed";
        if contains(output,'-- no solution --'), state="no_solution"; end
        if ~isempty(token)
            externalCost=str2double(token{1}); if strcmp(token{2},'N.A.'), bestTime=NaN; else, bestTime=str2double(token{2}); end
            routeText=string(strtrim(token{4})); routeValues=sscanf(char(routeText),'%d')'; state="solved";
        elseif status==0 && state~="no_solution"
            state="parse_failed";
        end
        internalCost=NaN; late=NaN; costError=NaN; gap=NaN; pass=false;
        if strcmp(state,'solved') && numel(routeValues)==instance.nCustomers
            [~,detail]=EvaluateTSPTWRoute(routeValues+1,instance,struct('latePenalty',1000));
            internalCost=detail.tourCost; late=detail.totalLate; costError=internalCost-externalCost;
            if isfinite(refCost), gap=(internalCost-refCost)/refCost; end
            pass=late<=1e-9 && abs(costError)<=1e-2;
        elseif strcmp(state,'solved')
            state="route_length_failed";
        end
        rows{end+1,1}={name,s,status,state,externalCost,internalCost,costError,late, ...
            refCost,gap,bestTime,wall,pass,routeText}; %#ok<AGROW>
    end
    fprintf('%s completed.\n',name);
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','seed','process_status','status','external_cost','internal_tour_cost', ...
    'cost_error','total_late','reference_cost','gap_to_bks','time_to_best', ...
    'wall_time','alignment_pass','route'});
results.summary=summary; results.files=files; results.seeds=seeds;
outDir=fullfile(root,'results');
writetable(summary,fullfile(outDir,'ddlns_formal_spb_summary.csv'));
save(fullfile(outDir,'ddlns_formal_spb_result.mat'),'results');
fprintf('Formal DD-LNS SPB completed: %d rows.\n',height(summary));
end
function file=FindExternalFile(root,name)
items=dir(fullfile(root,'**',char(name))); if isempty(items), error('找不到%s',name); end
file=fullfile(items(1).folder,items(1).name);
end
