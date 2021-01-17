
clear
close all

N = 39; % Number of files to iterate through
freq = 100; % Frequency of data
axisL = 20000; % Legnth of debugging axis
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
% Init
TrialDir = 'N:\IMPULSE GAIT ALGORITHM SELECTION\All Trials\' + string(freq) + 'Hz Trials';
WorkingDir = 'C:\Users\Ben\Desktop\Algorithms';
rmse_Seel = zeros(N,1); mae_Seel = zeros(N,1);
rmse_Allseits = zeros(N,1); mae_Allseits = zeros(N,1);
    
for i = 1:N
    cd (TrialDir);
    
    % To meet naming convention
%     if i < 10
%         index = '00' + string(i);
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
    
    cd (WorkingDir);    
    
    % Allseits Algorithm
    knee_angle_Allseits = Allseits(freq,gx_1,gy_1,gz_1,gx_2,gy_2,gz_2)';

    % Seel Algorithm
%     [~,~,~,knee_angle_Seel] = Seel(f,gx_1,gy_1,gz_1,ax_1,ay_1,az_1,...
%             gx_2,gy_2,gz_2,ax_2,ay_2,az_2,stepper_knee_angle,gait_stage);

    % Calculating Errors:
    % Differences
%     Seel_diff = knee_angle_Seel - stepper_knee_angle;
    Allseits_diff = knee_angle_Allseits - stepper_knee_angle;

    % Calculating RMSEs ------- CHECK THIS!
%     rmse_Seel(i) = sqrt(mean(Seel_diff.^2));
    rmse_Allseits(i) = sqrt(mean(Allseits_diff.^2));

    % Calculating MAEs ------- CHECK THIS!
%     mae_Seel(i) = mean(abs(Seel_diff));
    mae_Allseits(i) = mean(abs(Allseits_diff));
    
    % Error bars
    high = stepper_knee_angle + SLOP;
    low = stepper_knee_angle - SLOP;
    
    % Test plot
    figure(i+1)
    plot(stepper_knee_angle,'b-')
    hold on
    plot(high,'b:','LineWidth',0.5)
    plot(low,'b:','LineWidth',0.5)
%     plot(knee_angle_Seel(1:axisL),'g-')
    plot(knee_angle_Allseits,'r-')
    hold off
%     legend('Stepper','Gyro Only','Accel Only','CF','NumColumns',2)
    
end

% MAE_SEEL = mean(mae_Seel);
% RMSE_SEEL = mean(rmse_Seel);
MAE_ALLSEITS = mean(mae_Allseits);
RMSE_ALLSEITS = mean(rmse_Allseits);

figure(i+2)
bar(RMSE_ALLSEITS)

% figure(4)
% plot(a)
% hold on
% plot(g)
