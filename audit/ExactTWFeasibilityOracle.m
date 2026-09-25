function report = ExactTWFeasibilityOracle(model,cache,activeIDs,maxTime)
%EXACTTWFEASIBILITYORACLE 上层Routing-TW的MILP可行性审计
%   结论只针对当前LegCache提供的安全航段集合。
%   status: FEASIBLE / INFEASIBLE_CERTIFIED / UNKNOWN。

if nargin<4 || isempty(maxTime)
    maxTime = 30;
end
activeIDs = activeIDs(:)';
nCustomers = numel(activeIDs);
startNode = cache.startIndex;
homeNode = cache.homeIndex;

% 建立允许的有向航段变量：start->customer、customer->customer、customer->home。
edgeFrom = zeros(0,1);
edgeTo = zeros(0,1);
edgeCost = zeros(0,1);
for i = 1:cache.nNodes
    for j = 1:cache.nNodes
        if i==j || i==homeNode || j==startNode
            continue;
        end
        leg = cache.legs{i,j};
        if ~isempty(leg) && leg.isFeasible
            edgeFrom(end+1,1) = i; %#ok<AGROW>
            edgeTo(end+1,1) = j; %#ok<AGROW>
            edgeCost(end+1,1) = leg.distance; %#ok<AGROW>
        end
    end
end

nEdges = numel(edgeFrom);
sOffset = nEdges;
uOffset = nEdges+nCustomers;
nVars = nEdges+2*nCustomers;
if nEdges==0
    report = EmptyReport('INFEASIBLE_CERTIFIED');
    return;
end

edgeIndex = zeros(cache.nNodes,cache.nNodes);
for k = 1:nEdges
    edgeIndex(edgeFrom(k),edgeTo(k)) = k;
end

% 等式：每个客户一次进入、一次离开；起点一次离开；终点一次进入。
Aeq = sparse(0,nVars);
beq = zeros(0,1);
for customerNode = 2:(nCustomers+1)
    incoming = find(edgeTo==customerNode);
    outgoing = find(edgeFrom==customerNode);
    row = sparse(1,nVars);
    row(incoming) = 1;
    Aeq(end+1,:) = row; %#ok<AGROW>
    beq(end+1,1) = 1; %#ok<AGROW>
    row = sparse(1,nVars);
    row(outgoing) = 1;
    Aeq(end+1,:) = row; %#ok<AGROW>
    beq(end+1,1) = 1; %#ok<AGROW>
end
row = sparse(1,nVars);
row(find(edgeFrom==startNode)) = 1;
Aeq(end+1,:) = row;
beq(end+1,1) = 1;
row = sparse(1,nVars);
row(find(edgeTo==homeNode)) = 1;
Aeq(end+1,:) = row;
beq(end+1,1) = 1;

% 时间窗传播约束。
A = sparse(0,nVars);
b = zeros(0,1);
maxWindow = max(model.windows(activeIDs,2));
M = max(1e4,maxWindow+model.maxWaypointLift+model.speed*model.mapSize(1));
for k = 1:nEdges
    i = edgeFrom(k);
    j = edgeTo(k);
    leg = cache.legs{i,j};
    if j>=2 && j<=nCustomers+1
        localJ = j-1;
        row = sparse(1,nVars);
        if i==startNode
            row(sOffset+localJ) = -1;
            row(k) = M;
            rhs = M-model.startTime-leg.distance/model.speed;
        elseif i>=2 && i<=nCustomers+1
            localI = i-1;
            row(sOffset+localI) = 1;
            row(sOffset+localJ) = -1;
            row(k) = M;
            rhs = M-model.service(activeIDs(localI)) ...
                -leg.distance/model.speed;
        else
            continue;
        end
        A(end+1,:) = row; %#ok<AGROW>
        b(end+1,1) = rhs; %#ok<AGROW>
    end
end

% MTZ消除客户子回路。
for i = 2:(nCustomers+1)
    for j = 2:(nCustomers+1)
        if i==j
            continue;
        end
        k = edgeIndex(i,j);
        if k>0
            row = sparse(1,nVars);
            row(uOffset+i-1) = 1;
            row(uOffset+j-1) = -1;
            row(k) = nCustomers;
            A(end+1,:) = row; %#ok<AGROW>
            b(end+1,1) = nCustomers-1; %#ok<AGROW>
        end
    end
end

lb = -inf(nVars,1);
ub = inf(nVars,1);
lb(1:nEdges) = 0;
ub(1:nEdges) = 1;
lb(sOffset+(1:nCustomers)) = model.windows(activeIDs,1);
ub(sOffset+(1:nCustomers)) = model.windows(activeIDs,2);
lb(uOffset+(1:nCustomers)) = 1;
ub(uOffset+(1:nCustomers)) = nCustomers;

f = [edgeCost;zeros(2*nCustomers,1)];
intcon = 1:nEdges;
options = optimoptions('intlinprog','Display','off','MaxTime',maxTime, ...
    'RelativeGapTolerance',1e-4);

[solution,fval,exitflag,output] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub,options);
report.exitflag = exitflag;
report.output = output;
report.objective = fval;
report.nEdges = nEdges;
report.maxTime = maxTime;
report.referenceRoute = [];
report.referenceSchedule = [];

if exitflag==1 || exitflag==2
    report.status = 'FEASIBLE';
    x = solution(1:nEdges);
    report.referenceRoute = DecodeMILPRoute(x,edgeFrom,edgeTo,startNode,homeNode, ...
        activeIDs);
    report.referenceSchedule = solution(sOffset+(1:nCustomers));
elseif exitflag==-2
    report.status = 'INFEASIBLE_CERTIFIED';
else
    report.status = 'UNKNOWN';
end
end

function route = DecodeMILPRoute(x,edgeFrom,edgeTo,startNode,homeNode,activeIDs)
route = zeros(1,numel(activeIDs));
current = startNode;
for k = 1:numel(activeIDs)
    next = find(edgeFrom==current & x>0.5,1);
    if isempty(next)
        route = route(1:k-1);
        return;
    end
    current = edgeTo(next);
    route(k) = activeIDs(current-1);
end
if current==homeNode
    route = route(route>0);
end
end

function report = EmptyReport(status)
report.status = status;
report.exitflag = -2;
report.output = struct();
report.objective = inf;
report.nEdges = 0;
report.maxTime = 0;
report.referenceRoute = [];
report.referenceSchedule = [];
end
