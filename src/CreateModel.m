function model = CreateModel()
%CREATEMODEL 创建20客户的自主三维山地和障碍物场景
%   本文件不读取论文地图；地形和任务均为自主仿真数据。

%% 三维程序化山地
model.mapSize = [1000 800];
model.x = linspace(0,model.mapSize(1),201);
model.y = linspace(0,model.mapSize(2),161);
[model.X,model.Y] = meshgrid(model.x,model.y);
randomState = rng;
rng(2026,'twister');
noiseLarge = CreateRandomField(model.X,model.Y,9,8);
noiseMedium = CreateRandomField(model.X,model.Y,24,19);
noiseSmall = CreateRandomField(model.X,model.Y,58,45);
ridgeA = CurvedRidge(model.X,model.Y,250,420,0.30,55,170,150);
ridgeB = CurvedRidge(model.X,model.Y,680,560,-0.22,35,135,105);
ridgeC = CurvedRidge(model.X,model.Y,500,160,0.18,30,105,80);
ridgeTexture = 0.82+0.18*NormalizeField(noiseMedium);
ridgeTerm = (ridgeA+ridgeB+ridgeC).*ridgeTexture;
peakTerm = MountainPeak(model.X,model.Y,145,185,120,95,80) ...
    + MountainPeak(model.X,model.Y,300,570,175,115,95) ...
    + MountainPeak(model.X,model.Y,485,365,150,100,120) ...
    + MountainPeak(model.X,model.Y,650,690,125,105,75) ...
    + MountainPeak(model.X,model.Y,785,360,190,120,105) ...
    + MountainPeak(model.X,model.Y,900,630,110,85,90);
valleyTerm = CurvedValley(model.X,model.Y,50,355,0.28,42,72) ...
    + CurvedValley(model.X,model.Y,120,690,-0.18,35,58) ...
    + CurvedValley(model.X,model.Y,420,120,0.50,24,42);
model.terrainZ = 45+42*noiseLarge+16*noiseMedium+5*noiseSmall ...
    + ridgeTerm+peakTerm-valleyTerm;
model.terrainZ = ErosionRelaxation(model.terrainZ,12,0.10);
model.terrainZ = model.terrainZ-min(model.terrainZ(:));
model.terrainZ = 25+280*model.terrainZ/max(model.terrainZ(:));
rng(randomState);

%% 20个初始客户
model.depotXY = [80 400];
model.customerXY = [120 120;250 120;420 120;600 120;850 120; ...
                    100 300;240 300;420 300;620 300;900 300; ...
                    130 500;260 500;430 500;610 500;850 500; ...
                    120 700;270 700;450 700;680 700;900 700];
model.nCustomers = size(model.customerXY,1);
model.customerIDs = (1:model.nCustomers)';
model.startTime = 0;

%% 任务点统一位于安全飞行高度
model.cruiseHeight = 70;
model.flightAltitude = max(model.terrainZ(:))+model.cruiseHeight;
depotGroundZ = interp2(model.X,model.Y,model.terrainZ, ...
    model.depotXY(1),model.depotXY(2),'linear');
customerGroundZ = interp2(model.X,model.Y,model.terrainZ, ...
    model.customerXY(:,1),model.customerXY(:,2),'linear');
model.depot = [model.depotXY,model.flightAltitude];
model.homePosition = model.depot;
model.customerXYZ = [model.customerXY,repmat(model.flightAltitude,model.nCustomers,1)];
model.customerGroundXYZ = [model.customerXY,customerGroundZ];
model.nodeXYZ = [model.depot;model.customerXYZ];

%% 自主生成建筑障碍物
obstacleXYH = [330 410 130 110 125;570 575 150 120 145; ...
               760 270 135 150 120;475 175 105 120 155];
model.obstacles = repmat(struct('xMin',0,'xMax',0,'yMin',0, ...
    'yMax',0,'zMin',0,'zMax',0),size(obstacleXYH,1),1);
for k = 1:size(obstacleXYH,1)
    cx = obstacleXYH(k,1); cy = obstacleXYH(k,2);
    width = obstacleXYH(k,3); depth = obstacleXYH(k,4);
    height = obstacleXYH(k,5);
    ground = interp2(model.X,model.Y,model.terrainZ,cx,cy,'linear');
    model.obstacles(k).xMin = cx-width/2;
    model.obstacles(k).xMax = cx+width/2;
    model.obstacles(k).yMin = cy-depth/2;
    model.obstacles(k).yMax = cy+depth/2;
    model.obstacles(k).zMin = ground;
    model.obstacles(k).zMax = ground+height;
end
model.nObstacles = numel(model.obstacles);
model.obstacleSafety = 8;

%% 时间窗和飞行参数
% 20个客户使用可控的中等宽度时间窗，难度在实验脚本中单独调整。
earliest = 15+(0:model.nCustomers-1)'*2;
model.windows = [earliest,earliest+320];
model.service = repmat([8;10;8;10;12],4,1);
model.speed = 25;
model.minClearance = 40;
model.safetySamples = 60;
model.nWaypointsPerLeg = 2;
model.lateralScale = 0.28;
model.maxWaypointLift = 190;
model.latePenalty = 100;
model.terrainPenalty = 5000;
model.obstaclePenalty = 15000;
model.smoothPenalty = 1.5;
end

function field = CreateRandomField(X,Y,nx,ny)
x0 = linspace(min(X(:)),max(X(:)),nx); y0 = linspace(min(Y(:)),max(Y(:)),ny);
field = interp2(x0,y0,randn(ny,nx),X,Y,'spline');
field = NormalizeField(field);
end
function field = NormalizeField(field)
field = field-mean(field(:)); field = field/(std(field(:))+eps);
end
function ridge = CurvedRidge(X,Y,cx,cy,slope,width,height,xWidth)
centerLine = cy+slope*(X-cx)+35*sin((X-cx)/120);
ridge = height*exp(-(Y-centerLine).^2/(2*width^2));
ridge = ridge.*exp(-(X-cx).^2/(2*xWidth^2));
end
function valley = CurvedValley(X,Y,cx,cy,slope,width,depth)
centerLine = cy+slope*(X-cx)+28*sin((X-cx)/145);
valley = depth*exp(-(Y-centerLine).^2/(2*width^2));
valley = valley.*exp(-(X-(cx+420)).^2/(2*500^2));
end
function peak = MountainPeak(X,Y,cx,cy,height,sigmaX,sigmaY)
peak = height*exp(-((X-cx).^2/(2*sigmaX^2)+(Y-cy).^2/(2*sigmaY^2)));
end
function Z = ErosionRelaxation(Z,nIterations,alpha)
kernel = ones(3,3)/9;
for k = 1:nIterations
    smoothZ = conv2(Z,kernel,'same');
    localRelief = abs(Z-smoothZ);
    threshold = prctile(localRelief(:),70);
    mask = localRelief>threshold;
    Z(mask) = (1-alpha)*Z(mask)+alpha*smoothZ(mask);
end
end

