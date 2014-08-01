%draw the stewart platform
function draw_stewart(robot, twist, Q)
%basic angular unit conversion
deg2rad = pi/180;

P = robot{1};
B = robot{2};
gamma = robot{3};
L_m = robot{4};
L_s = robot{5};
Theta_min = robot{6};
Theta_max = robot{7};

%apply transform to top.  This assumes that the initial pos_top is at
%twist(0 0 0 0 0 0) relative to the base.
%orientation:
%rotation around x axis:
P = P * [[1     0               0               ];
         [0     cos(twist(4))   -sin(twist(4))  ];
         [0     sin(twist(4))   cos(twist(4))   ]];

%rotation around y axis:
P = P * [[cos(twist(5))     0   sin(twist(5))   ];
         [0                 1   0               ];
         [-sin(twist(5))    0   cos(twist(5))   ]];

%rotation around z axis:
P = P * [[cos(twist(6))     -sin(twist(6))  0   ];
         [sin(twist(6))     cos(twist(6))   0   ];
         [0                 0               1   ]];

%position:
%translation along x:
P(:,1) = P(:,1)+twist(1);

%translation along y:
P(:,2) = P(:,2)+twist(2);

%translation along z:
P(:,3) = P(:,3)+twist(3);

%close the base and platform polygons to help with drawing them
B(7,:) = B(1,:);
P(7,:) = P(1,:);

leg = [];

for i = 1:6,
    leg(i,:,:) = [[P(i,:)];[B(i,:)]];
end

%draw the top platform
plot3(P(:,1),P(:,2),P(:,3))
grid on
axis square
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

%set the volume of the plot
axis([-0.2 0.2 -0.2 0.2 0 0.4])
hold on
%draw the base
plot3(B(:,1),B(:,2),B(:,3))

for i=1:6,
    
    %draw the hypotenuse of each leg in red
    plot3(leg(i,:,1),leg(i,:,2),leg(i,:,3),'r')
   
    M = []; %This is the point of the elbow
    
    M(1) = L_m*(cos(Q(i)) * sin(gamma(i))) + B(i,1);
    M(2) = L_m*(-cos(Q(i)) * cos(gamma(i))) + B(i,2);
    M(3) = L_m*(sin(Q(i))) + B(i,3);
    
    Leg_master = [B(i,:);M];    %link from base to elbow
    Leg_slave = [M;P(i,:)];     %link from elbow to platform
    
    %draw the master and slave links red if they are not the right size,
    %otherwise they are black. 
    if (abs(norm(B(i,:)-M) - L_m) - 0.0001)
        plot3(Leg_master(:,1),Leg_master(:,2),Leg_master(:,3),'k')
    else
        plot3(Leg_master(:,1),Leg_master(:,2),Leg_master(:,3),'r')
    end
    
    if (abs(norm(P(i,:)-M) - L_s) < 0.0001)
        plot3(Leg_slave(:,1),Leg_slave(:,2),Leg_slave(:,3),'k')
    else
        plot3(Leg_slave(:,1),Leg_slave(:,2),Leg_slave(:,3),'r')
        
    end
       
end
%make hypotenuse of leg 1 green for reference:
i=1;
plot3(leg(i,:,1),leg(i,:,2),leg(i,:,3),'g')

hold off

