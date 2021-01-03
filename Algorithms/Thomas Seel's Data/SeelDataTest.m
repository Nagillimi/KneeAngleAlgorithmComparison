%% Loading Thomas Seel's Data for testing
clear
close

gyrodata = readmatrix('gyrodata_prosthesis_walking.csv')';

% gx_1 = gyrodata(:,1);
% gy_1 = gyrodata(:,2);
% gz_1 = gyrodata(:,3);
% gx_2 = gyrodata(:,4);
% gy_2 = gyrodata(:,5);
% gz_2 = gyrodata(:,6);

% Thomas said his j-vectors came to:
% j_upper = [-0.1,0.5,-0.9]
% j_lower = [0,1,0.1]