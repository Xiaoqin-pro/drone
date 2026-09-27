function results = RunDDLNSAlignmentCheck
%RUND DLNSALIGNMENTCHECK 核对DD-LNS与本项目SPB评价器的路线/目标语义

root=fileparts(fileparts(mfilename('fullpath'))); setup;
exe=fullfile(root,'external_baselines','ijcai_22_DDLNS-main','source-code', ...
    'solver-ddlns','target','release','examples','tsptw.exe');
externalDir=fullfile(root,'external_baselines','ijcai_22_DDLNS-main','benchmarks','tsptw');
localDir=fullfile(root,'data','tsp_tw_raw','SolomonPotvinBengio');
files=["rc_201.1.txt","rc_201.3.txt","rc_202.1.txt","rc_204.2.txt", ...
    "rc_207.3.txt","rc_208.3.txt"];
rows=cell(0,1);
if ~isfile(exe), error('找不到DD-LNS binary：%s',exe); end
for k=1:numel(files)
    externalFile=FindExternalFile(externalDir,files(k));
    localFile=fullfile(localDir,files(k));
    command=sprintf('"%s" solve -f "%s" -w 100 -t 30 -s 20211105 -H',exe,externalFile);
    [status,output]=system(command);
    token=regexp(output,'\|\s*lns\s*\|\s*[^|]+\|\s*([0-9.]+)\s*\|[^|]*\|[^|]*\|[^|]*\|\s*(.*)','tokens','once');
    if isempty(token)
        rows{end+1,1}={files(k),status,NaN,NaN,NaN,NaN,"parse_failed",string(output)}; %#ok<AGROW>
        continue;
    end
    externalCost=str2double(token{1}); routeText=strtrim(token{2}); routeValues=sscanf(routeText,'%d')';
    instance=ReadTSPTWInstance(localFile); internalRoute=routeValues+1;
    if numel(internalRoute)==instance.nCustomers
        [~,detail]=EvaluateTSPTWRoute(internalRoute,instance,struct('latePenalty',1000));
        internalCost=detail.tourCost; late=detail.totalLate; costError=internalCost-externalCost;
        pass=late<=1e-9 && abs(costError)<=1e-2;
        rows{end+1,1}={files(k),status,externalCost,internalCost,costError,late, ...
            string(pass),string(strjoin(string(routeValues),' '))}; %#ok<AGROW>
    else
        rows{end+1,1}={files(k),status,externalCost,NaN,NaN,NaN,"route_length_failed", ...
            string(routeText)}; %#ok<AGROW>
    end
    fprintf('%s completed.\n',files(k));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','process_status','external_cost','internal_tour_cost','cost_error', ...
    'internal_total_late','pass','route'});
results.summary=summary; results.files=files;
writetable(summary,fullfile(root,'results','ddlns_alignment_check.csv'));
save(fullfile(root,'results','ddlns_alignment_check.mat'),'results');
fprintf('DD-LNS alignment check completed: %d rows.\n',height(summary));
end

function file=FindExternalFile(root,name)
items=dir(fullfile(root,'**',char(name)));
if isempty(items), error('DD-LNS benchmark中找不到%s',name); end
file=fullfile(items(1).folder,items(1).name);
end


