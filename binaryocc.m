clear all
close all
%% The generation of the map

image = imread('tecnico_grid.png');
imwrite(image, 'tecnico_grid.pgm');
imagePGM = imread('tecnico_grid.pgm');

imageBW = imagePGM > 254;
imageWB = 1 - imageBW;

map = binaryOccupancyMap(imageWB);

%% The initialization of the MCL

mcl = monteCarloLocalization;

%% LidarScan
lidar = rangeSensor;
lidar.HorizontalAngle = [-pi/2 pi/2];
lidar.Range = [0 30];

%% odometryMotionModel

odo = odometryMotionModel;
odo.Noise = [0.2 0.2 0.2 0.2];

%% LikelihoodFieldSensorModel

lf = likelihoodFieldSensorModel;
lf.SensorLimits = [0 30];
lf.Map = map;

%% Helper methods
visualizationHelper = ExampleHelperMCLVisualization(map);

%% Turning on the localization


mcl.SensorModel = lf;
mcl.MotionModel = odo;

mcl.ParticleLimits = [500 5000];
mcl.GlobalLocalization = false;
mcl.UseLidarScan = true;
mcl.InitialCovariance = 0;
mcl.ResamplingInterval = 1;

%% Creating the trajectory

button = 1;
k = 1;
while button==1
    [x(k),y(k),button] = ginput(1);
    plot(x(k),y(k),'r+')
    k = k + 1;
end

vehiclePose = [transpose(x) transpose(y)  zeros(length(x), 1)];
vehiclePose(length(vehiclePose), :) = [];

positionSize = size(vehiclePose,1);

for i=1:positionSize
    if i ~= positionSize
        current = vehiclePose(i, :);
        next = vehiclePose(i+1, :);
        vehiclePose(i, 3) = atan2(next(2)-current(2),next(1)-current(1));
    end
end

mcl.InitialPose = vehiclePose(1, :);
numUpdates = length(vehiclePose)-1;
i = 1;
hold on;
while i < numUpdates
    pose = vehiclePose(i, :);
    [ranges, angles] = lidar(pose, map);
    scan = lidarScan(ranges, angles);
    [isUpdated, estimatedPose, estimatedCovariance] = mcl(pose, scan);
    if isUpdated 
        i = i + 1;
        plotStep(visualizationHelper, mcl, estimatedPose, scan.Ranges, scan.Angles, i);
    end
end

