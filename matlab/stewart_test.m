%stewart positioning tester

%basic angular unit conversion
deg2radX = pi/180;

%load the stewart platform model:
robot = modelStewartPlatform;
robot_str = robot{8};

TransX = 0.0; %meters
TransY = -0.0; %meters
TransZ = 0.25; %meters
Roll = 0 * deg2radX; %degrees
Pitch = 0 * deg2radX; %degrees
Yaw = 0 * deg2radX; %degrees
twist = [TransX TransY TransZ Roll Pitch Yaw];

[success_cf,Q] = stewart_ikcf(robot, twist)
draw_stewart(robot,twist,Q)
title(robot_str);

