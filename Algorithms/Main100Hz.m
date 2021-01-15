
clear
close all

N = 20; % Number of files to iterate through
axisL = 15000; % Legnth of debugging axis
SLOP = 2.89*2; % Stepper Slop in degrees (x2 steppers...)

%% Load Seel Test Data
% j_upper = [-0.1,0.5,-0.9];
% j_lower = [0,1,0.1];

% cd ("C:\Users\Ben\Desktop\Algorithms\Thomas Seel's Data");
% gyrodata = readmatrix('gyrodata_prosthesis_walking.csv');
% 
% gx_1 = gyrodata(:,1);
% gy_1 = gyrodata(:,2);
% gz_1 = gyrodata(:,3);
% gx_2 = gyrodata(:,4);
% gy_2 = gyrodata(:,5);
% gz_2 = gyrodata(:,6);
% 
% cd ('C:\Users\Ben\Desktop\Algorithms');
% 
% % Obtaining j vectors, following Seel's 2014 study
% gyrodata = [gx_1,gy_1,gz_1,gx_2,gy_2,gz_2]';
% [j1,j2,x_j] = localJ(gyrodata,true);

%% Run Gait Trial Data

rmse_Seel = zeros(N,1); mae_Seel = zeros(N,1);
for i = 1:N
    cd ('N:\IMPULSE GAIT ALGORITHM SELECTION\All Trials\100Hz Trials');
    
    % To meet naming convention
    if i < 10
        index = '0' + string(i);
    else
        index = string(i);
    end
    fileString = 'Trial_' + index + '.csv';
    
    dataFile = readmatrix(fileString);
    dataFile = dataFile(7:end,:);
    
    packet = dataFile(:,1);
    time = dataFile(:,2);
    time_delta = dataFile(:,3);
    
    % Accel data is rotated since calibration wasn't applied on a
    % flat surface, but instead a vertical surface.
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
    
    cd ('C:\Users\Ben\Desktop\Algorithms');
    
    % Allseits Algorithm
%     knee_angle_Allseits = Allseits(gx_1,gy_1,gz_1,gx_2,gy_2,gz_2);

    % Seel Algorithm
    [acceldata,a,g,knee_angle_Seel] = Seel(gx_1,gy_1,gz_1,ax_1,ay_1,az_1,...
            gx_2,gy_2,gz_2,ax_2,ay_2,az_2,stepper_knee_angle,gait_stage);

    % Calculating Errors:
    % Differences
    Seel_diff = g(1:axisL) - stepper_knee_angle(1:axisL);
%     Allseits_diff(i) = knee_angle_Allseits - stepper_knee_angle;

    % Calculating RMSEs ------- CHECK THIS!
    rmse_Seel(i) = sqrt(mean(Seel_diff.^2));
%     rmse_Allseits(i) = mean(sqrt((Allseits_diff).^2));

    % Calculating MAEs ------- CHECK THIS!
    mae_Seel(i) = mean(abs(Seel_diff));
%     mae_Allseits(i) = mean(abs(Allseits_diff));

    % Plots 
    
    high = stepper_knee_angle(1:axisL) + SLOP;
    low = stepper_knee_angle(1:axisL) - SLOP;
    
    
    figure(i+1)
    % Test plot

    plot(stepper_knee_angle(1:axisL),'b-')
    hold on
    plot(high,'b:','LineWidth',0.5)
    plot(low,'b:','LineWidth',0.5)
%     plot(g(1:axisL),'r-')
%     plot(a(1:axisL),'g-')
    plot(knee_angle_Seel(1:axisL))
    hold off
%     legend('Stepper','Gyro Only','Accel Only','CF','NumColumns',2)
    
end

MAE_SEEL = mean(mae_Seel);
RMSE_SEEL = mean(rmse_Seel);

% figure(4)
% plot(a)
% hold on
% plot(g)
