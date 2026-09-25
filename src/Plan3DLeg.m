function leg = Plan3DLeg(from,to,model)
%PLAN3DLEG 为一对任务点生成一条三维安全航段
%   下层使用有限候选航路点进行确定性规划，并缓存航段结果。

xyDelta = to(1:2)-from(1:2);
xyLength = norm(xyDelta);
if xyLength<eps
    perpendicular = [0 0];
else
    perpendicular = [-xyDelta(2),xyDelta(1)]/xyLength;
end

offsetList = [0 -0.25 0.25 -0.50 0.50 -0.75 0.75];
liftList = [0 40 80 120 160 200];
candidates = cell(0,1);
candidates{end+1} = [from;to]; %#ok<AGROW>

for offset = offsetList(2:end)
    for lift = liftList
        p1 = from+(to-from)/3;
        p2 = from+2*(to-from)/3;
        lateralOffset = offset*xyLength*perpendicular;
        p1(1:2) = ClampXY(p1(1:2)+lateralOffset,model);
        p2(1:2) = ClampXY(p2(1:2)+lateralOffset,model);
        p1(3) = p1(3)+lift;
        p2(3) = p2(3)+lift;
        candidates{end+1} = [from;p1;p2;to]; %#ok<AGROW>
    end
end

bestIndex = 1;
bestKey = [inf inf inf];
for k = 1:numel(candidates)
    metric = EvaluatePolyline(candidates{k},model);
    if metric.isFeasible
        key = [0 metric.distance metric.smoothness];
    else
        key = [1 metric.totalViolation metric.distance];
    end
    if IsLexicographicallySmaller(key,bestKey)
        bestIndex = k;
        bestKey = key;
    end
end

metric = EvaluatePolyline(candidates{bestIndex},model);
leg.points = candidates{bestIndex};
leg.distance = metric.distance;
leg.terrainViolation = metric.terrainViolation;
leg.obstacleViolation = metric.obstacleViolation;
leg.totalViolation = metric.totalViolation;
leg.minClearance = metric.minClearance;
leg.smoothness = metric.smoothness;
leg.isFeasible = metric.isFeasible;
end

function xy = ClampXY(xy,model)
xy(1) = min(max(xy(1),min(model.x)),max(model.x));
xy(2) = min(max(xy(2),min(model.y)),max(model.y));
end

function metric = EvaluatePolyline(points,model)
distance = 0;
terrainViolation = 0;
obstacleViolation = 0;
minClearance = inf;

for s = 1:size(points,1)-1
    from = points(s,:);
    to = points(s+1,:);
    t = linspace(0,1,model.safetySamples)';
    samples = from+t.*(to-from);
    groundZ = interp2(model.X,model.Y,model.terrainZ, ...
        samples(:,1),samples(:,2),'linear');
    clearance = samples(:,3)-groundZ;
    terrainDeficit = max(0,model.minClearance-clearance);
    terrainViolation = terrainViolation+mean(terrainDeficit);
    minClearance = min(minClearance,min(clearance));

    for k = 1:model.nObstacles
        obs = model.obstacles(k);
        insideXY = samples(:,1)>=obs.xMin-model.obstacleSafety ...
            & samples(:,1)<=obs.xMax+model.obstacleSafety ...
            & samples(:,2)>=obs.yMin-model.obstacleSafety ...
            & samples(:,2)<=obs.yMax+model.obstacleSafety;
        verticalDeficit = max(0,obs.zMax+model.obstacleSafety-samples(:,3));
        if any(insideXY)
            obstacleViolation = obstacleViolation+ ...
                mean(verticalDeficit(insideXY));
        end
    end
    distance = distance+norm(to-from);
end

smoothness = CalculateSmoothness(points);
metric.distance = distance;
metric.terrainViolation = terrainViolation;
metric.obstacleViolation = obstacleViolation;
metric.totalViolation = terrainViolation+obstacleViolation;
metric.minClearance = minClearance;
metric.smoothness = smoothness;
metric.isFeasible = metric.totalViolation<=1e-9;
end

function smoothness = CalculateSmoothness(points)
segments = diff(points,1,1);
segments = reshape(segments,[],3);
lengths = sqrt(sum(segments.^2,2));
valid = lengths>eps;
segments = segments(valid,:);
validLengths = lengths(valid);
if size(segments,1)<2
    smoothness = 0;
    return;
end
segments = bsxfun(@rdivide,segments,validLengths);
turn = 1-sum(segments(1:end-1,:).*segments(2:end,:),2);
heightChange = abs(diff(points(:,3),2));
smoothness = sum(turn.^2)+0.01*sum(heightChange);
end

function result = IsLexicographicallySmaller(a,b)
result = false;
for i = 1:numel(a)
    if a(i)<b(i)-1e-10
        result = true;
        return;
    elseif a(i)>b(i)+1e-10
        return;
    end
end
end
