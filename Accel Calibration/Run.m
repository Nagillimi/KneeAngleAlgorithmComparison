% Accel Error Measurement
clear 
close

cd ('N:\IMPULSE GAIT ALGORITHM SELECTION\Accelerometer Calibration Errors')

angle_results = zeros(3,10);
for i = 1:10
    foo = string(i);
    bar = 'Results ' + foo + '.csv';
    this = readmatrix(bar);
    
    angle_results(1:3,i) = this(1:3,7);
end

horiz_mean = mean(angle_results(1,:));
UM7_1_mean = mean(angle_results(2,:));
UM7_2_mean = mean(angle_results(3,:));

UM7_1_angle = abs(horiz_mean) - abs(UM7_1_mean);
UM7_2_angle = abs(horiz_mean) - abs(UM7_2_mean);