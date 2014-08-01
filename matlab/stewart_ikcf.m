%draw the stewart platform
function [success,Q_out] = stewart_ikcf(robot, twist)

P = robot{1};   %Initially the nominal zero pose of attachment points for the moving platform
B = robot{2};   %The pose of the attachment points of the base
gamma = robot{3};

R = robot{4};   %This is the length of the master link of each arm (servo to elbow)
D = robot{5};   %This is the length of the slave link of each arm (elbow to moving platform)
Theta_min = robot{6};   %minimum allowable angle for servo travel
Theta_max = robot{7};   %maximum allowable angle for servo travel

rev_mask = [1 1 1 -1 -1 -1];

twist = twist .* rev_mask;

%apply transform to top.  This assumes that the initial pos_top is at
%twist(0 0 0 0 0 0) relative to the base.
%orientation:
%rotation around z axis:
P = P * [[cos(twist(6))     -sin(twist(6))  0   ];
         [sin(twist(6))     cos(twist(6))   0   ];
         [0                 0               1   ]];
%rotation around y axis:
P = P * [[cos(twist(5))     0   sin(twist(5))   ];
         [0                 1   0               ];
         [-sin(twist(5))    0   cos(twist(5))   ]];
%rotation around x axis:
P = P * [[1     0               0               ];
         [0     cos(twist(4))   -sin(twist(4))  ];
         [0     sin(twist(4))   cos(twist(4))   ]];



%position:
%translation along x:
P(:,1) = P(:,1)+twist(1);

%translation along y:
P(:,2) = P(:,2)+twist(2);

%translation along z:
P(:,3) = P(:,3)+twist(3);

%Iterate through all six leg lengths to identify angles for the servos
Q_out = zeros(1,6);
success = 0;    %lets us track if successful angles are found for each leg

for i=1:6,
   
    L = norm(P(i,:) - B(i,:));  %distacne from the base to platform 
    %attachment points.  this is the hypotenuse of the triangle between 
    %the points of the base attachment point (B), platform attachment 
    %point (P) and elbow (M, not seen in this script)
    
    %The derivation of these is complicated...
    a = 2*R*(P(i,3) - B(i,3));
    b = 2*R*(sin(gamma(i))*(P(i,1) - B(i,1)) - cos(gamma(i))*(P(i,2) - B(i,2)));
    c = (L^2 - D^2 + R^2);
   
    %calculate the angle for each joint. 
    theta = asin(c/sqrt(a^2 + b^2)) - atan2(b,a);
    
    %if the hypotenuse is greater than the master + slave link lengths, it
    %is impossible to form a triangle, and we get imaginary components in
    %our joint angles. check for and reject solutions with imaginary
    %components, and also check for violations of joint limits.
    if ((theta >= Theta_min) && (theta <= Theta_max) && (real(theta) == theta))
        Q_out(i) = theta;
        success = success + 1;  %if all angles are valid, success = 6.  
    else
        Q_out(i) = 0;
    end
    
end

