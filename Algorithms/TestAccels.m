clear
close

% File #
i = 2;

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

% Accel data is rotated 90deg, since calibration wasn't applied on a
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

% Collect individual datasets
gyrodata = [gx_1,gy_1,gz_1,gx_2,gy_2,gz_2]';
acceldata_ = [ax_1,ay_1,az_1,ax_2,ay_2,az_2]';

% Correct acceldata (from calibration error)
acceldata = CorrectAccelBias(acceldata_,stepper_knee_angle,gait_stage);

%% Test Plots

M = 1;
N = 15562;

% Before

figure(1)
subplot(4,1,1)
plot((hip_stepper(M:N)-780).*0.18)
hold on
plot((knee_stepper(M:N)-390).*0.18)
title('Stepper Angles');
legend('hip','knee','NumColumns',2)

subplot(4,1,2)
plot(ax_1(M:N))
hold on
plot(ax_2(M:N))
title('AX Thigh & Shank');

subplot(4,1,3)
plot(ay_1(M:N))
hold on
plot(ay_2(M:N))
title('AY Thigh & Shank');

subplot(4,1,4)
plot(az_1(M:N))
hold on
plot(az_2(M:N))
title('AZ Thigh & Shank');

legend('Thigh','Shank','NumColumns',2)

% After

figure(2)
subplot(4,1,1)
plot((hip_stepper(M:N)-780).*0.18)
hold on
plot((knee_stepper(M:N)-390).*0.18)
title('Stepper Angles');
legend('hip','knee','NumColumns',2)

subplot(4,1,2)
plot(acceldata(1,M:N))
hold on
plot(acceldata(4,M:N))
title('AX Thigh & Shank');

subplot(4,1,3)
plot(acceldata(2,M:N))
hold on
plot(acceldata(5,M:N))
title('AY Thigh & Shank');

subplot(4,1,4)
plot(acceldata(3,M:N))
hold on
plot(acceldata(6,M:N))
title('AZ Thigh & Shank');

legend('Thigh','Shank','NumColumns',2)
