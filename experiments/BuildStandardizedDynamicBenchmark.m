function BuildStandardizedDynamicBenchmark
%BUILDSTANDARDIZEDDYNAMICBENCHMARK 将Solomon实例转换为标准化动态单UAV数据
scriptDir=fileparts(mfilename('fullpath'));
root=fileparts(scriptDir);
rawDir=fullfile(root,'data','solomon_raw','selected');
outDir=fullfile(root,'data','standardized_dynamic');
if ~exist(outDir,'dir'), mkdir(outDir); end

sourceNames={'r101','c101','rc101'};
nCustomers=20;
revealFraction=0.20;
eventTimeFraction=0.50;

for s=1:numel(sourceNames)
    name=sourceNames{s};
    solomon=ReadSolomonInstance(fullfile(rawDir,[name '.txt']));
    baseModel=CreateModel();
    model=ConvertToSingleUAVInstance(solomon,baseModel,nCustomers);
    hiddenCount=round(nCustomers*revealFraction);
    hiddenIDs=(nCustomers-hiddenCount+1):nCustomers;
    activeIDs=setdiff(model.customerIDs,hiddenIDs,'stable');
    event.time=eventTimeFraction*solomon.depotRow(6);
    event.type='add';
    event.customerIDs=hiddenIDs;
    for k=1:numel(hiddenIDs)
        id=hiddenIDs(k);
        event.customers(k).id=id; %#ok<AGROW>
        event.customers(k).xy=model.customerXY(id,:); %#ok<AGROW>
        event.customers(k).window=model.windows(id,:); %#ok<AGROW>
        event.customers(k).service=model.service(id); %#ok<AGROW>
    end
    instance.sourceBenchmark=name;
    instance.sourceName=solomon.name;
    instance.customerCount=nCustomers;
    instance.revealFraction=revealFraction;
    instance.eventTimeFraction=eventTimeFraction;
    instance.model=model;
    instance.activeIDs=activeIDs;
    instance.hiddenIDs=hiddenIDs;
    instance.event=event;
    save(fullfile(outDir,[name '_20_dynamic.mat']),'instance');
    fprintf('Built %s: %d active + %d hidden customers.\n', ...
        name,numel(activeIDs),numel(hiddenIDs));
end
end
