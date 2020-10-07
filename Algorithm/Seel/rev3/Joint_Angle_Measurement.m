% IMU-Based Joint Angle Measurement for Gait Analysis
%
%

clear

%for i = 000:499
    
    Trial = readmatrix('Trial_000.csv','Range','A7:T40000');
    
    
    
    % Call function to determine the joint axes and position,
    % assign to j & o
    [j1,j2,ej,o1,o2,eo] = Joint_Axes_and_Position(Trial);

%end

disp(j1);
disp(j2);
disp(o1);
disp(o2);
