%load the vision library:
addpath('rvctools')
startup_rvc

%Target is assumed to be four circular points arranged in a square with a
%specified edge length:
tgtEdge = 0.05  %meters
circleRadius = 0.02 %meters

zOffset = 0.25 %m (desired offset from target to camera:

%adjusting this scale changes running speed a lot...
Scale = 0.5   %image resize scale

%gain for IBVS:
lambda = 0.3

%configuration of the camera:
focalLength = 6;        %mm
frameSize = [640 480];  %H and V detectors
detPitch = 6;           %microns on each side (square)
detPitchm = detPitch / 1000000     %meters
sensorSize = frameSize * detPitchm	%meters
focalLength = focalLength / 1000	%meters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%parameters for how big and small of circles to look for
Rnom = (focalLength * circleRadius) / (detPitchm * zOffset);

%Scale the Rmin and Rmax values by the image resize scale
Rmin = floor(Rnom * 0.4 * Scale);
Rmax = floor(Rnom * 1.25 * Scale + 0.5);

%calculate the central camera model
cam = CentralCamera('focal', focalLength, 'sensor', sensorSize, 'resolution', frameSize);

%create a model of the test target:
%square with edges 10cm long (0.1m) that is translated 0.3m in the Z
%direction
pStar = mkgrid(2, tgtEdge, 'T', transl(0, 0, zOffset))

%uv_pStar is what we want pStar to look like at our desired range in detector
%units
uv_pStar = cam.project(pStar)

%create a window to display the camera's view
h = imshow(zeros(480, 640, 'uint8'));
%include red marks indicating the desired target locations in the frame
l = viscircles(uv_pStar', [Rnom Rnom Rnom Rnom],'EdgeColor','r');
drawnow

%read in the image
img_rgb = imread('circle_test.jpg');

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

%Display values to screen:
uv_pxy_ordered
uv_radii_ordered
zEst

%The error of rotation around y (yaw) is based on the focal
%length, detector size and number of pixels of error in the
%horizontal (u) direction.
%u_error = cam.u0 - uv_p_center(1);
%and around x (pitch)
%v_error = cam.v0 - uv_p_center(2);

%this following should be inside the control loop:
e = uv_pStar - uv_pxy_ordered';
e = e(:)

%The jacobian of the pont
J = visjac_p(cam, uv_pxy_ordered', zEst)
%desired (camera) image velocity
v = lambda * pinv(J) * e

%convert velocity into differential motion at the camera (end
%-effector)
Tdelta = trnorm(delta2tr(v))



