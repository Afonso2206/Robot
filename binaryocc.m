
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
mcl.GlobalLocalization = true;
mcl.ResamplingInterval = 1;

%% LidarScan
lidar = rangeSensor;
lidar.HorizontalAngle = [-pi/2 pi/2];
lidar.Range = [0 20];
vehiclePose = [360 230 0];

[ranges, angles] = lidar(vehiclePose, map);

scan = lidarScan(ranges, angles);

%% odometryMotionModel

odo = odometryMotionModel;
odo.Noise = [0.2 0.2 0.2 0.2];

%% LikelihoodFieldSensorModel

lf = likelihoodFieldSensorModel;
lf.SensorLimits = [0.5 20];
lf.Map = map;
lf.SensorPose = vehiclePose;

%% Combining all the different models

mcl.SensorModel = lf;
mcl.MotionModel = odo;


%% Turning on the localization

[isUpdated, estimatedPose, covariance] = mcl(vehiclePose, scan);

%% Get the final particles with their weights






