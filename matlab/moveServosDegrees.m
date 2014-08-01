function moveServosDegrees(degs,port)
%example usage: degs = zeros(1,6) + 60;
%               moveServosDegrees(degs,'COM15');

%adjust the center point of so that the servos move from 0-90 rather than
%-45 to 45 degrees. 
%degs input range is 0-90, output range is -45 - 45 because the zero mark 
%on the servo is at 45 degrees on the physical platform:
degs = degs - 45;

%Mask used to reverse the direction of some servos because of how they are
%mounted
rev_mask = [1 -1 -1 -1 1 1];

%Apply the reverse mask
degs = degs .* rev_mask;


ser1 = serial(port);
set(ser1, 'InputBufferSize', 2048);
set(ser1, 'BaudRate', 9600);
set(ser1, 'DataBits', 8);
set(ser1, 'Parity', 'none');
set(ser1, 'StopBits', 1);

fopen(ser1);    %initialize

for s=1:size(degs,2)
    
    x = degs(s) * 10.5 + 1500; %map degrees to time units where 1500 
    %represents 0 degrees (well, 45 degrees on the physical platform) 
    if (x ~= 0)
        x = max(600,x);
        x = min(2400,x);
        x = 4*x;
    end
    
    AA = [170,12,4,s, bin2dec(num2str(flip(bitget(x,1:7)))), bin2dec(num2str(flip(bitget(x,8:14))))];
    %!!!Note, this is complicated...it is best to compare the results of
    %what matlab generates here with that of ROS to make sure that they
    %agree.  The flipping of byte order is messy along with string
    %conversion.  The website:
    %http://air.imag.fr/index.php/Pololu_Maestro_Servo_Controller#Robot_Operating_System
    %has a better way I think.  
   
    fwrite(ser1, AA);
    
end

fclose(ser1);
delete(ser1);
end