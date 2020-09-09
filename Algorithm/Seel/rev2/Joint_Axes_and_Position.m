% Thomas Seel Least-squares method for joint axes estimation, 2012/2014
% Implemented by Ben Milligan Aug, 2020
% Returns joint axes j1,j2 and positions o1,o2 from 6d inertial data
% 
% Change to read directly from csv files -> readmatrix()
% Change to use every third to tenth sample -> 3*k or 10*k
% Change to lsqminnorm(de/dx,e) instead of pinv(de/dx)*e, it's faster

% param:
%   Trial is a matrix of the entire dataset
% return:
%   j1 joint axis seen from imu1
%   j2 joint axis seen from imu2
%   o1 position of imu1
%   o2 position of imu2
function [j1,j2,o1,o2] = Joint_Axes_and_Position(Trial)
%--------------------------------------------------------------------------
    % Joint Vector Estimations {j1,j2}
    % Total length of trial dataset
    len = length(Trial.TIME);

    % Initial guess for psi1, psi2, theta1, theta2
    % rand assigns values in R{0,1}
    psi1 = rand;    psi2 = rand;    theta1 = rand;    theta2 = rand;

    % Initialize the solution vector x
    x = [psi1,psi2,theta1,theta2]';

    % Initial values for end case
    tol = 0.01;    ej = [len,1];     k = 1;

    % Read all gyro values from file {x,y,z}
    g1 = [Trial.G1X, Trial.G1Y, Trial.G1Z];
    g2 = [Trial.G2X, Trial.G2Y, Trial.G2Z];

    % Declare the gyro derivatives
    Trial.gx1dot = zeros(size(transfer));
    Trial.gy1dot = zeros(size(transfer));
    Trial.gz1dot = zeros(size(transfer));

    Trial.gx2dot = zeros(size(transfer));
    Trial.gy2dot = zeros(size(transfer));
    Trial.gz2dot = zeros(size(transfer));

    % Compute the gyro derivatives with third order approx
    for i = 3:len-2
        Trial.gx1dot(i) = (gx1(i-2) - 8*gx1(i-1) + 8*gx1(i+1) - gx1(i+2))/12;
        Trial.gy1dot(i) = (gy1(i-2) - 8*gy1(i-1) + 8*gy1(i+1) - gy1(i+2))/12;
        Trial.gz1dot(i) = (gz1(i-2) - 8*gz1(i-1) + 8*gz1(i+1) - gz1(i+2))/12;

        Trial.gx2dot(i) = (gx2(i-2) - 8*gx2(i-1) + 8*gx2(i+1) - gx2(i+2))/12;
        Trial.gy2dot(i) = (gy2(i-2) - 8*gy2(i-1) + 8*gy2(i+1) - gy2(i+2))/12;
        Trial.gz2dot(i) = (gz2(i-2) - 8*gz2(i-1) + 8*gz2(i+1) - gz2(i+2))/12;
    end

    % Combine vectors to a Nx3 matrix per unit
    g1dot = [gx1dot,gy1dot,gz1dot];    g2dot = [gx2dot,gy2dot,gz2dot];

    % Update loop for joint vector estimation
    % Returns latest {j1,j2} once the e vector value < tol
    while ej(k) > tol
        % Conversion from roll & azimuth coord to rectangle coord
        j1 = [cos(psi1)*cos(theta1),cos(psi1)*sin(theta1),sin(psi1)];
        j2 = [cos(psi2)*cos(theta2),cos(psi2)*sin(theta2),sin(psi2)];

        % Compute error vector e
        ej(k,1) = norm(cross(g1(k,:),j1)) - norm(cross(g2(k,:),j2));

        % Compute de/dx
        dej_dj1 = ( cross(cross(g1(k,:),j1),j1) ) / ( norm(cross(g1(k,:),j1)) );
        dej_dj2 = ( cross(cross(g2(k,:),j2),j2) ) / ( norm(cross(g2(k,:),j2)) );

        % Jacobian
        dej_dx = [dej_dj1',dej_dj2'];

        % Update solution using pseudoinverse
        x = x - pinv(dej_dx) * ej;

        % Enumerate loop counter
        k = k + 1;
    end

    %--------------------------------------------------------------------------
    % Offset Vector Estimations {o1,o2}

    % Initial guess for o1 and o2
    % rand assigns values in R{0,1} for 2x1 vector {psi,theta}'
    psi_o1 = rand;     theta_o1 = rand;     psi_o2 = rand;     theta_o2 = rand; 

    % Initialize the solution vector y
    y = [psi_o1, psi_o2, theta_o1, theta_o2]';

    % Initial values for end case
    eo = [len,1];     k = 1;

    % Read all accel values from file {x,y,z}
    % size Nx3
    a1 = [Trial000.A1X, Trial000.A1Y, Trial000.A1Z]';
    a2 = [Trial000.A2X, Trial000.A2Y, Trial000.A2Z]';

    % Update loop for offset axis estimation
    while eo(k) > tol
        % Conversion from roll & azimuth coord to rectangle coord
        o1 = [cos(psi_o1)*cos(theta_o1),cos(psi_o1)*sin(theta_o1),sin(psi_o1)];
        o2 = [cos(psi_o2)*cos(theta_o2),cos(psi_o2)*sin(theta_o2),sin(psi_o2)];

        % Compute rotational accelerations
        %rot1 = cross(g1(k,:),cross(g1(k,:),o1)) + cross(g1dot(k,:),o1);
        %rot2 = cross(g2(k,:),cross(g2(k,:),o2)) + cross(g2dot(k,:),o2);
        rot1 = rot(g1(k,:),g1dot(k,:),o1);
        rot2 = rot(g2(k,:),g2dot(k,:),o2);

        % Compute error vector
        eo(k,1) = norm(a1(k,:) - rot1) - norm(a2(k,:) - rot2);

        % Compute de/dx
        deo_do1 = rotT( g1(k,:), g1dot(k,:), (a1(k,:) - rot1) ) / norm(a1(k,:) - rot1);
        deo_do2 = rotT( g2(k,:), g2dot(k,:), (a2(k,:) - rot2) ) / norm(a2(k,:) - rot2);

        % Jacobian
        deo_dx = [deo_do1',deo_do2'];

        % Update solution using psuedoinverse
        y = y - pinv(deo_dx) * eo;

        % Enumerate loop counter
        k = k + 1;
    end
end