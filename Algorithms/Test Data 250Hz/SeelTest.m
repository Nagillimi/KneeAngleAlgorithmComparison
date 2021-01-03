% breaks trialXX into vector components
%
% TODO: 
% - Shift to storing NO VARIABLES, and just use Trial00.
%   to access functions.
% -

transfer = Trial00.TRANSFER;
time = Trial00.TIME;

gx1 = Trial00.G1X;
gy1 = Trial00.G1Y;
gz1 = Trial00.G1Z;
ax1 = Trial00.A1X;
ay1 = Trial00.A1Y;
az1 = Trial00.A1Z;

gx2 = Trial00.G2X;
gy2 = Trial00.G2Y;
gz2 = Trial00.G2Z;
ax2 = Trial00.A2X;
ay2 = Trial00.A2Y;
az2 = Trial00.A2Z;

% 38926x3 arrays
g1 = [gx1,gy1,gz1]; g2 = [gx2,gy2,gz2];

% Declare the gyro derivatives
Trial00.gx1dot = zeros(size(transfer));
Trial00.gy1dot = zeros(size(transfer));
Trial00.gz1dot = zeros(size(transfer));

Trial00.gx2dot = zeros(size(transfer));
Trial00.gy2dot = zeros(size(transfer));
Trial00.gz2dot = zeros(size(transfer));

% Init the gyro derivatives via third order approx
% Note: from (i-2) to (i+2)
for i = 3:length(transfer)-2
    Trial00.gx1dot(i) = (gx1(i-2) - 8*gx1(i-1) + 8*gx1(i+1) - gx1(i+2))/12;
    Trial00.gy1dot(i) = (gy1(i-2) - 8*gy1(i-1) + 8*gy1(i+1) - gy1(i+2))/12;
    Trial00.gz1dot(i) = (gz1(i-2) - 8*gz1(i-1) + 8*gz1(i+1) - gz1(i+2))/12;
    
    Trial00.gx2dot(i) = (gx2(i-2) - 8*gx2(i-1) + 8*gx2(i+1) - gx2(i+2))/12;
    Trial00.gy2dot(i) = (gy2(i-2) - 8*gy2(i-1) + 8*gy2(i+1) - gy2(i+2))/12;
    Trial00.gz2dot(i) = (gz2(i-2) - 8*gz2(i-1) + 8*gz2(i+1) - gz2(i+2))/12;
end

% Declare the e vector. 6 = # gyro datasets
e = ones(6,1);

%--------------------------------------------------------------------------
% Newton minimization to obtain const j1 and j2 vectors
% x_new = x - H'(x)*gradient(g(x))

% Provide initial guess for j vectors in spherical coords
% (PHI,THETA)
j1Guess = [0,0]; j2Guess = [0,0];
phi1 = j1Guess(1); theta1 = j1Guess(2); phi2 = j2Guess(1); theta2 = j2Guess(2);

% 3x3 by 3x1 = 3x1
% May be (theta,phi)
j1_est = ShperToRec(phi1, theta1)*[1;0;0];
j2_est = ShperToRec(phi2, theta2)*[1;0;0];

% Init j vectors with spherical coord
% j1_est = [cos(phi1)*cos(theta1), -sin(phi1), -cos(phi1)*sin(theta1); ...
%     sin(phi1)*cos(theta1), cos(phi1), -sin(phi1)*sin(theta1); sin(theta1), 0, cos(theta1)]*[1;0;0];
% 
% j2_est = [cos(phi2)*cos(theta2), -sin(phi2), -cos(phi2)*sin(theta2); ...
%     sin(phi2)*cos(theta2), cos(phi2), -sin(phi2)*sin(theta2); sin(theta2), 0, cos(theta2)]*[1;0;0];

% Init the e vector
for i = 1:length(e)
    % 1x3 by 3x1 = 1x1 (scalar)
    e(i)= 1.*( norm(cross(g1(i,:),j1_est))...
    - norm(cross(g2(i,:),j2_est)) );
end

%--------------------------------------------------------------------------
