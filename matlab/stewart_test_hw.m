%stewart positioning tester
%port = 'COM5'; %use this for windows
%for Linux, do the following (you will need to install drivers for the
%maestro from the pololu website.  Then because matlab doesn't recognize
%the ACM0 style ports, we need a symbolic link into a ttySXXX name that
%matlab can work with...
%!sudo ln -s /dev/ttyACM0 /dev/ttyS101
port = '/dev/ttyS101';

%basic angular unit conversion
deg2radX = pi/180;

%load the stewart platform model:
robot = modelStewartPlatform;
robot_str = robot{8};

TransX = -0.0; %meters
TransY = -0.00; %meters
TransZ = 0.25; %meters
Roll = 0 * deg2radX; %degrees
Pitch = 0 * deg2radX; %degrees
Yaw = 0 * deg2radX; %degrees
twist = [TransX TransY TransZ Roll Pitch Yaw];
[success,Q] = stewart_ikcf(robot, twist)
if (success == 6)   %only execute on HW if the calculated angles are known to be good. 
    moveServosDegrees(round(Q / deg2radX),port);
    draw_stewart(robot,twist,Q)
    title(robot_str);
end
