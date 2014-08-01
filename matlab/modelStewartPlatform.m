function robot = modelStewartPlatform

%basic angular unit conversion
deg2radX = pi/180;

%define the connection points on the base and top plate with respect to the
%world frame of reference, which is the center of the base plate.
B = [];
P = [];

%parameters that configure the stewart platform geometry
alpha_b = 15*deg2radX; %angle between leg root and center between two neighboring legs
alpha_b2 = -30*deg2radX; %angle of departure of the master link arm from line between neighboring legs
alpha_t = 10*deg2radX; %angle between leg top and center between two neighboring legs
radius_b = 0.1035; %0.11; %radius of the base where legs attach to base
radius_t = 0.05; %radius of the top where legs attach to the top
Theta_min = 0;%-pi/4; %minimum angle for the master link (L_m) from base plane
Theta_max = pi/2;%3*pi/4; %maximum angle for the master link (L_m) from base plane
%config option 1:
%top_offset_angle = 0; %60 degrees = 2*pi/6:  offset of top plate from base
%L_m = 0.1657; %length of the master link, driven by servo (m)
%config option 2:
top_offset_angle = 2*pi/6; %60 degrees = 2*pi/6:  offset of top plate from base
L_m = 0.1683; %length of the master link, driven by servo (m)

L_s = 0.2156; %Length of the slave link from master link to top platform (m).

gamma = [];

%create the baseline geometry
for i = 1:3,
    
    % base points
    angle_m_b = (2*pi/3)* (i-1) - alpha_b;
    angle_p_b = (2*pi/3)* (i-1) + alpha_b;
    gamma(2*i-1) = angle_m_b + alpha_b2;
    gamma(2*i) = angle_p_b - alpha_b2;
    B(2*i-1,:) = radius_b* [cos(angle_m_b), sin(angle_m_b), 0.0];
    B(2*i,:) = radius_b* [cos(angle_p_b), sin(angle_p_b), 0.0];
    
    % top points (with a 60 degree offset)
    angle_m_t = (2*pi/3)* (i-1) - alpha_t + top_offset_angle;
    angle_p_t = (2*pi/3)* (i-1) + alpha_t + top_offset_angle;
    P(2*i-1,:) = radius_t* [cos(angle_m_t), sin(angle_m_t), 0.0];
    P(2*i,:) = radius_t* [cos(angle_p_t), sin(angle_p_t), 0.0];
    
end
P = circshift(P,1); %This is used to remap leg attachments...
%done creating baseline geometry

gamma = gamma + pi/2; %this makes gamma the angle of the axis of rotation
%rather than the angle of departure for the master link

M = []; %This is the point of the elbow

M(1) = L_m*(cos(0) * sin(gamma(1))) + B(1,1);
M(2) = L_m*(-cos(0) * cos(gamma(1))) + B(1,2);
M(3) = L_m*(sin(0)) + B(1,3);

L_m_calc = norm(M-B(1,:))
L_s_calc = norm(P(1,:)-M)


title_str=sprintf('Master Link = %0.4f m, Slave Link = %0.4f m \nBase: Radius = %0.2f m, Alpha = %0.2f deg \nTop: Radius = %0.2f m, Alpha = %0.2f deg\nTheta min = %0.2f, Theta max = %0.2f', L_m, L_s, radius_b, alpha_b/deg2radX, radius_t, alpha_t/deg2radX, Theta_min, Theta_max);
robot = {P B gamma L_m L_s Theta_min Theta_max title_str};