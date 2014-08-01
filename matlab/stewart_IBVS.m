% Image based visual servoing using stewart manipulator
% Todd Danko


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%configurable values:
%port = 'COM15';
%!sudo ln -s /dev/ttyACM0 /dev/ttyS101
port = '/dev/ttyS101';

%basic angular unit conversion
deg2radX = pi/180;

%Target is assumed to be four circular points arranged in a square with a
%specified edge length:
tgtEdge = 0.05;  %meters
circleRadius = 0.02; %meters

zOffset = 0.35; %m (desired offset from target to camera:

%parameters for how big and small of circles to look for
Rmin = 4.4 / zOffset * 2 ;%* 2;
Rmax = 12 / zOffset * 2;

%adjusting this scale changes running speed a lot...
Scale = .5;   %image resize scale


%lambda is the gain, applied to the jacobian to calculate velocity
%to approach the desired pose
lambda = 0.4;

%configuration of the camera:
focalLength = 4;        %mm
frameSize = [640 480];  %H and V detectors
detPitch = 6;           %microns on each side (square)
detPitchm = detPitch / 1000000;     %meters
sensorSize = frameSize * detPitchm;	%meters
focalLength = focalLength / 1000;	%meters
a_val = (focalLength * tgtEdge) / (detPitchm * zOffset);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

c = clock;
c = fix(c);

%hand to camera transform.  Identity for now, but this affords options for
%other more complicated configurations
%hTc = transl(0, 0, 0);
hTc = rpy2tr(0, pi/2, pi/2);

%and the camera to hand transform is the inverse of hTc:
cTh = inv(hTc);

running = 1;
tgtRunning = 0;

%Scale the Rmin and Rmax values by the image resize scale
Rmin = floor(Rmin * Scale);
Rmax = floor(Rmax * Scale + 0.5);

%Load the arm model
robot = modelStewartPlatform;


%define the desired pose of the arm based on initial q:
%Tdesired: 

Tdesired = [[ 1  0  0 0];
            [ 0  1  0 0];
            [ 0  0  1 0.25];
            [ 0  0  0 1]];

%transform from base to hand
bTh = Tdesired;
twist = tr2delta(bTh);
[success,Q] = stewart_ikcf(robot, twist);
moveServosDegrees(round(Q / deg2radX),port);

%setup the camera source (firefly color)
try,
    dc1394('capture_stop', camera);
    dc1394('video_set_transmission', camera, 0);
    dc1394('camera_free', camera);
    clear camera;
end

camera = dc1394('init_camera');
err = dc1394('capture_setup', camera, 4, 'DEFAULT');
err = dc1394('video_set_transmission', camera, 1);

%calculate the central camera model
cam = CentralCamera('focal', focalLength, 'sensor', sensorSize, 'resolution', frameSize);

%create a model of the test target:
%square with edges 10cm long (0.1m) that is translated 0.3m in the Z
%direction
pStar = mkgrid(2, tgtEdge, 'T', transl(0, 0, zOffset));

%uv_pStar is what we want pStar to look like at our desired range in detector
%units
uv_pStar = cam.project(pStar);

%create a window to display the camera's view
h = imshow(zeros(480, 640, 'uint8'));
%include red marks indicating the desired target locations in the frame
l = viscircles(uv_pStar', [10 10 10 10],'EdgeColor','r');
drawnow

%initialize errors to 0:
u_error = 0;
v_error = 0;

%start timer
tic;

while(running)
    %grab a frame
    frame = dc1394('capture_dequeue_latest', camera);
    
    %debayer the image
    img_rgb = dc1394('bayer', frame.image, 'NEAREST', 'RGGB');
    
    %reduce the image size (speeds up processing)
    img_small = imresize(img_rgb, Scale);
    
    %detect the target circles and return their center locations
    [uv_p_raw, radii_raw] = imfindcircles(img_small,[Rmin Rmax],'ObjectPolarity','dark');
    
    %rescale the detected target circle centers based on image scale factor
    uv_pxy = uv_p_raw / Scale;
    uv_radii = radii_raw / Scale;
    
    %display the captured image with circles drawn over the detected target
    %circles
    set(h, 'CData', img_rgb);
    c = viscircles(uv_pxy, radii_raw/Scale,'EdgeColor','b');
    drawnow expose
    delete(c);
    
    %proceed only if four points were detected
    if (size(uv_pxy,1) == 4)
        
        %reorganize the points to better match the template
        uv_pxy_ordered = zeros(size(uv_pxy));
        uv_radii_ordered = zeros(size(uv_radii));
        
        %calculate the center point across all of the target circles
        uv_p_center = [mean(uv_pxy(:,1)) mean(uv_pxy(:,2))];
        
        for Pt= 1:4
            if ((uv_pxy(Pt,1) < uv_p_center(1)) && (uv_pxy(Pt,2) < uv_p_center(2)))
                uv_pxy_ordered(1,:) = uv_pxy(Pt,:);
                uv_radii_ordered(1) = uv_radii(Pt);
            elseif ((uv_pxy(Pt,1) < uv_p_center(1)) && (uv_pxy(Pt,2) > uv_p_center(2)))
                uv_pxy_ordered(2,:) = uv_pxy(Pt,:);
                uv_radii_ordered(2) = uv_radii(Pt);
            elseif ((uv_pxy(Pt,1) > uv_p_center(1)) && (uv_pxy(Pt,2) > uv_p_center(2)))
                uv_pxy_ordered(3,:) = uv_pxy(Pt,:);
                uv_radii_ordered(3) = uv_radii(Pt);
            elseif ((uv_pxy(Pt,1) > uv_p_center(1)) && (uv_pxy(Pt,2) < uv_p_center(2)))
                uv_pxy_ordered(4,:) = uv_pxy(Pt,:);
                uv_radii_ordered(4) = uv_radii(Pt);
            end
        end
        
        %estimate the range to each point knowing the angular resolution
        %and detected radius, along with known target sizes
        zEst = zeros(size(uv_radii_ordered));
        
        for Pt= 1:4
            zEst(Pt) = (circleRadius*focalLength)/(uv_radii_ordered(Pt)*detPitchm);
        end
        
        %The error of rotation around y (yaw) is based on the focal
        %length, detector size and number of pixels of error in the
        %horizontal (u) direction.
        u_error = cam.u0 - uv_p_center(1);
        %and around x (pitch)
        v_error = cam.v0 - uv_p_center(2);
        
        %this following should be inside the control loop:
        e = uv_pStar - uv_pxy_ordered';
        e = e(:);
        
        %The jacobian of the pont
        J = visjac_p(cam, uv_pxy_ordered', zEst);
        
        %desired (camera) image velocity
        v = lambda * pinv(J) * e;
        
        %convert velocity into differential motion at the camera (end
        %-effector)
        Tdelta = trnorm(delta2tr(v))
        
        %the transform from the base to the camera is simply the
        %combination of base to hand then hand to camera
        bTc = trnorm(bTh * hTc);
        
        %we need to solve for the motion of the camera relative to the base
        %rather than relative to the camera (Tdelta is relative to the
        %camera), so the desired base to camera transform is the old base
        %to camera transform combined with Tdelta
        bTcStar = trnorm(bTc * Tdelta);
        
        %to drive the arm, we need to know where to command the hand, not
        %the camera, so we find the desire base to hand transform by
        %combining the desired base to camera transform with the camera to
        %hand transform resulting in a desired base to hand transform
        bThStar = trnorm(bTcStar * cTh); %this is correct (p457)
        
        %calculate the IK to reach the new goal from base:
        twist = tr2delta(bThStar);
        [success,Q] = stewart_ikcf(robot, twist)
        if (success == 6)
            moveServosDegrees(round(Q / deg2radX),port);
            
            %transform from base to hand (just forward kinematics from arm,
            bTh = bThStar;            
        end
        

        
    end
    %rate = 1/toc;
    %tic;
    %display(rate);
    timeStamp = toc;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%use stop_vs.m to unload the camera
%stop camera:
% dc1394('capture_stop', camera);
% dc1394('video_set_transmission', camera, 0);
% dc1394('camera_free', camera);
% clear camera;


