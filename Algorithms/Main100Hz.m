
clear
close

%% Load Seel Test Data
% j_upper = [-0.1,0.5,-0.9];
% j_lower = [0,1,0.1];

cd ("/Users/BenMilligan/Desktop/Algorithm Coding/Thomas Seel's Data");
gyrodata = readmatrix('gyrodata_prosthesis_walking.csv');

gx_1 = gyrodata(:,1);
gy_1 = gyrodata(:,2);
gz_1 = gyrodata(:,3);
gx_2 = gyrodata(:,4);
gy_2 = gyrodata(:,5);
gz_2 = gyrodata(:,6);

cd ('/Users/BenMilligan/Desktop/Algorithm Coding');

%% Load Gait Trial Data

i = 10;
% for i = 1:500
    cd ('/Volumes/ARCHIVE/IMPULSE GAIT ALGORITHM SELECTION/All Trials/100Hz Trials');
    dataFile = readmatrix('Trial_10.csv');
    dataFile = dataFile(7:end,:);
    
    packet = dataFile(:,1);
    time = dataFile(:,2);
    time_delta = dataFile(:,3);
    gx_1 = dataFile(:,4);
    gy_1 = dataFile(:,5);
    gz_1 = dataFile(:,6);
    ax_1 = dataFile(:,7);
    ay_1 = dataFile(:,8);
    az_1 = dataFile(:,9);
    gx_2 = dataFile(:,10);
    gy_2 = dataFile(:,11);
    gz_2 = dataFile(:,12);
    ax_2 = dataFile(:,13);
    ay_2 = dataFile(:,14);
    az_2 = dataFile(:,15);
    hip_stepper =  dataFile(:,16);
    knee_stepper = dataFile(:,17);
    stepper_knee_angle = dataFile(:,18);
    gait_stage = dataFile(:,19);
    impulse_hit = dataFile(:,20);
    
    cd ('/Users/BenMilligan/Desktop/Algorithm Coding');
    
    
%% Test Plots

plot(ax_1)

%% Allseits Algorithm

[knee_angle_Allseits,error_Allseits] = Allseits(gx_1,gy_1,gz_1,gx_2,gy_2,gz_2);


%% Seel Algorithm

[knee_angle_Seel,error_Seel] = Seel(gx_1,gy_1,gz_1,ax_1,ay_1,az_1,gx_2,gy_2,gz_2,ax_2,ay_2,az_2);

% end