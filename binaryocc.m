



%% The generation of the map

image = imread('tecnico_grid.png');
imwrite(image, 'tecnico_grid.pgm');
imagePGM = imread('tecnico_grid.pgm');

imageBW = imagePGM > 254;
imageWB = 1 - imageBW;

map = binaryOccupancyMap(imageWB);

%% The initialization of the MCL

mcl = monteCarloLocalization;
mcl.UseLidarScan = true;
mcl.InitialPose = [360 230 0];
mcl.InitialCovariance = 0;
mcl.ResamplingInterval = 1;

%% LidarScan
lidar = rangeSensor;
lidar.HorizontalAngle = [-pi/2 pi/2];
lidar.Range = [0 30];

%% odometryMotionModel

odo = odometryMotionModel;
odo.Noise = [0.2 0.2 0.2 0.2];

%% LikelihoodFieldSensorModel

lf = likelihoodFieldSensorModel;
lf.SensorLimits = [0.5 40];
lf.Map = map;

%% Helper methods
visualizationHelper = ExampleHelperMCLVisualization(map);

%% Turning on the localization
vehiclePose = [
    360 230 0;
    360 230 5;
    364 235 60;
    360 240 110;
    360 245 60;
    360 260 90;
    350 240 270;
    330 230 -45;
    331 229 -50;
    332 227 -30;
    334 226 -15;
    337 225 0;
    340 225 0;
    345 225 0;
    ];

mcl.SensorModel = lf;
mcl.MotionModel = odo;

numUpdates = 2;
i = 1;
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






