function results = RunDDLNSInitialAlignment
%RUND DLNSINITIALALIGNMENT 用作者初始解核对6个代表性SPB实例
root=fileparts(fileparts(mfilename('fullpath'))); setup;
exe=fullfile(root,'external_baselines','ijcai_22_DDLNS-main','source-code','solver-ddlns','target','release','examples','tsptw.exe');
bench=fullfile(root,'external_baselines','ijcai_22_DDLNS-main','benchmarks','tsptw');
initFile=fullfile(bench,'initial_solutions','SolomonPotvinBengio.txt');
localDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
files=["rc_201.1.txt","rc_201.3.txt","rc_202.1.txt","rc_204.2.txt","rc_207.3.txt","rc_208.3.txt"];
seeds=1:5; rows=cell(0,1);
lines=readlines(initFile);
for k=1:numel(files)
    name=files(k); line=lines(startsWith(strtrim(lines),name));
    tokens=regexp(char(line(1)),'^\S+\s+[-+]?\d*\.?\d+\s+\d+\s+(.*)$','tokens','once');
    if isempty(tokens), error('无法解析初始路线：%s',name); end
    initialRouteText=strtrim(tokens{1});
    externalFile=FindExternalFile(bench,name); localFile=fullfile(localDir,name);
    for s=1:numel(seeds)
        command=sprintf('"%s" solve -f "%s" -w 100 -t 5 -s %d --solution "%s" -H', ...
            exe,externalFile,20211105+s,initialRouteText);
        [status,output]=system(command);
        token=regexp(output,'\|\s*lns\s*\|\s*[^|]+\|\s*([0-9.]+)\s*\|[^|]*\|[^|]*\|[^|]*\|\s*(.*)','tokens','once');
        instance=ReadTSPTWInstance(localFile);
        if isempty(token)
            rows{end+1,1}={name,s,status,NaN,NaN,NaN,NaN,"parse_failed",string(output)}; %#ok<AGROW>
            continue;
        end
        externalCost=str2double(token{1}); routeValues=sscanf(strtrim(token{2}),'%d')';
        if numel(routeValues)==instance.nCustomers
            [~,detail]=EvaluateTSPTWRoute(routeValues+1,instance,struct('latePenalty',1000));
            rows{end+1,1}={name,s,status,externalCost,detail.tourCost, ...
                detail.tourCost-externalCost,detail.totalLate, ...
                string(detail.totalLate<=1e-9 && abs(detail.tourCost-externalCost)<=1e-2), ...
                string(strtrim(token{2}))}; %#ok<AGROW>
        else
            rows{end+1,1}={name,s,status,externalCost,NaN,NaN,NaN,"route_length_failed",string(token{2})}; %#ok<AGROW>
        end
    end
    fprintf('%s initial-solution alignment completed.\n',name);
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','seed','process_status','external_cost','internal_tour_cost','cost_error', ...
    'internal_total_late','pass','route'});
results.summary=summary; results.files=files; results.seeds=seeds;
writetable(summary,fullfile(root,'results','ddlns_initial_alignment.csv'));
save(fullfile(root,'results','ddlns_initial_alignment.mat'),'results');
fprintf('DD-LNS initial alignment completed: %d rows.\n',height(summary));
end
function file=FindExternalFile(root,name)
items=dir(fullfile(root,'**',char(name))); if isempty(items), error('找不到%s',name); end
file=fullfile(items(1).folder,items(1).name);
end
