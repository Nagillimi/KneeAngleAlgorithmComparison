
clear
close all

s_file = 381; % First file to iterate from
f_file = 400; % Last file to iterate to
files_to_ignore = 0;%[40,97,166,215,232,326,338,348,363,365,366]';
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
Tot_files = f_file - s_file; 
read_file = true; files_read = 0;
% rmse_Seel = zeros(Tot_files,1); mae_Seel = zeros(Tot_files,1);
rmse_Allseits = zeros(Tot_files,1); mae_Allseits = zeros(Tot_files,1);
    
for i = s_file:f_file
    read_file = true;
    
    cd (TrialDir);
    
    % To meet naming convention
%     if i < 10
%         index = '00' + string(i);
    if i < 10
        index = '0' + string(i);
    else
        index = string(i);
    end
    
    % Reject sus files
    for j = 1:length(files_to_ignore)
        if i == files_to_ignore(j)
            read_file = false;
        end
    end
    
    if read_file == true
        fileString = 'Trial_' + index + '.csv';

        dataFile = readmatrix(fileString);
        dataFile = dataFile(7:end,:);

        packet = dataFile(:,1);
        time = dataFile(:,2);
        time_delta = dataFile(:,3);

        % Read file
        gx_1 = (dataFile(:,4));
        gy_1 = (dataFile(:,5));
        gz_1 = (dataFile(:,6));
        ax_1 = (dataFile(:,7));
        ay_1 = (dataFile(:,8));
        az_1 = (dataFile(:,9));
        gx_2 = (dataFile(:,10));
        gy_2 = (dataFile(:,11));
        gz_2 = (dataFile(:,12));
        ax_2 = (dataFile(:,13));
        ay_2 = (dataFile(:,14));
        az_2 = (dataFile(:,15));
        hip_stepper =  dataFile(:,16);
        knee_stepper = dataFile(:,17);
        stepper_knee_angle = dataFile(:,18);
        gait_stage = dataFile(:,19);
        impulse_hit = dataFile(:,20);
        
        % Change dir
        cd (WorkingDir); 
        %%
        %  Remove NaN | Inf components
        gx_1 = gx_1(~isnan(gx_1) | ~isinf(gx_1));
        gy_1 = gy_1(~isnan(gy_1) | ~isinf(gy_1));
        gz_1 = gz_1(~isnan(gz_1) | ~isinf(gz_1));
        ax_1 = ax_1(~isnan(ax_1) | ~isinf(ax_1));
        ay_1 = ay_1(~isnan(ay_1) | ~isinf(ay_1));
        az_1 = az_1(~isnan(az_1) | ~isinf(az_1));
        gx_2 = gx_2(~isnan(gx_2) | ~isinf(gx_2));
        gy_2 = gy_2(~isnan(gy_2) | ~isinf(gy_2));
        gz_2 = gz_2(~isnan(gz_2) | ~isinf(gz_2));
        ax_2 = ax_2(~isnan(ax_2) | ~isinf(ax_2));
        ay_2 = ay_2(~isnan(ay_2) | ~isinf(ay_2));
        az_2 = az_2(~isnan(az_2) | ~isinf(az_2));
        
%         gy_1 = zero_order_hold_remove_nan(gy_1);
%         gz_1 = zero_order_hold_remove_nan(gz_1);
%         ax_1 = zero_order_hold_remove_nan(ax_1);
%         ay_1 = zero_order_hold_remove_nan(ay_1);
%         az_1 = zero_order_hold_remove_nan(az_1);
%         gx_2 = zero_order_hold_remove_nan(gx_2);
%         gy_2 = zero_order_hold_remove_nan(gy_2);
%         gz_2 = zero_order_hold_remove_nan(gz_2);
%         ax_2 = zero_order_hold_remove_nan(ax_2);
%         ay_2 = zero_order_hold_remove_nan(ay_2);
%         az_2 = zero_order_hold_remove_nan(az_2);
%         %%
%         replace_index = isnan(gx_2) + isinf(gx_2);
%     
%     for j = 1:length(gx_2)
%         if replace_index(j) > 0
%             % Zero order hold (past value)
%             gx_2(j) = gx_2(j-1);
%         else
%             gx_2(j) = gx_2(j);
%         end
%     end
%     
%     for j = 1:length(gy_2)
%         if replace_index(j) > 0
%             % Zero order hold (past value)
%             gy_2(j) = gy_2(j-1);
%         else
%             gy_2(j) = gy_2(j);
%         end
%     end
        
%%
        
        

        % Allseits Algorithm
        knee_angle_Allseits = Allseits(freq,gx_1,gy_1,gz_1,gx_2,gy_2,gz_2)';

        % Seel Algorithm
    %     [~,~,~,knee_angle_Seel] = Seel(f,gx_1,gy_1,gz_1,ax_1,ay_1,az_1,...
    %             gx_2,gy_2,gz_2,ax_2,ay_2,az_2,stepper_knee_angle,gait_stage);

        % Calculating Errors:
        % Differences
    %     Seel_diff = knee_angle_Seel - stepper_knee_angle;
        Allseits_diff = knee_angle_Allseits - stepper_knee_angle(1:length(knee_angle_Allseits));

        % Calculating RMSEs ------- CHECK THIS!
    %     rmse_Seel(i-(s_file-1)) = sqrt(mean(Seel_diff.^2));
        rmse_Allseits(i-(s_file-1)) = sqrt(mean(Allseits_diff.^2));

        % Calculating MAEs ------- CHECK THIS!
    %     mae_Seel(i-(s_file-1)) = mean(abs(Seel_diff));
        mae_Allseits(i-(s_file-1)) = mean(abs(Allseits_diff));

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
    
        files_read = files_read + 1;
    end
    
    clear dataFile packet time time_delta gx_1 gy_1 gz_1 ax_1 ay_1 az_1 ...
        gx_2 gy_2 gz_2 ax_2 ay_2 az_2 hip_stepper knee_stepper ...
        stepper_knee_angle gait_stage impulse_hit knee_angle_Allseits ...
        knee_angle_Seel high low
end

% MAE_SEEL = mean(mae_Seel);
% RMSE_SEEL = mean(rmse_Seel);
MAE_ALLSEITS = sum(mae_Allseits)/files_read;
RMSE_ALLSEITS = sum(rmse_Allseits)/files_read;

% figure(i+2)
% bar(RMSE_ALLSEITS)

% figure(4)
% plot(a)
% hold on
% plot(g)
