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
    % Read variables from the passed csv file
    transfer = Trial(:,1);  
    g1 = [Trial(:,4), Trial(:,5), Trial(:,6)];
    g2 = [Trial(:,10), Trial(:,11), Trial(:,12)];
    a1 = [Trial(:,7), Trial(:,8), Trial(:,9)];
    a2 = [Trial(:,13), Trial(:,14), Trial(:,15)];
%--------------------------------------------------------------------------
    % Joint Vector Estimations {j1,j2}
    % Total length of trial dataset
    len = length(transfer);

    % Initial guess for psi1, psi2, theta1, theta2
    % rand assigns values in R{0,1}
    psi1 = rand;    psi2 = rand;    theta1 = rand;    theta2 = rand;
    % Initialize the solution vector x
    x = [psi1,psi2,theta1,theta2];

    % Initial values for end case
    tol = 0.1;    ej = [len,3];    
    
    % Declare the gyro derivatives
    g1dot = zeros(size(g1));
    g2dot = zeros(size(g2));

    % Compute the gyro derivatives with third order approx
    for i = 3:len-2
        for j = 1:3
            g1dot(i,j) = (g1(i-2,j) - 8*g1(i-1,j) + 8*g1(i+1,j) - g1(i+2,j))/12;
            g2dot(i,j) = (g2(i-2,j) - 8*g2(i-1,j) + 8*g2(i+1,j) - g2(i+2,j))/12;
        end
    end

    % Update loop for joint vector estimation
    % Returns latest {j1,j2} once the e vector value < tol
    for k = 1:1000
        % Conversion from roll & azimuth coord to rectangle coord
        j1 = [cos(psi1)*cos(theta1),cos(psi1)*sin(theta1),sin(psi1)];
        j2 = [cos(psi2)*cos(theta2),cos(psi2)*sin(theta2),sin(psi2)];

        % Compute error vector e
        ej(k,:) = norm(cross(g1(k,:),j1)) - norm(cross(g2(k,:),j2));

        % Compute de/dx
        dej_dj1 = ( cross(cross(g1(k,:),j1),j1) ) / ( norm(cross(g1(k,:),j1)) );
        dej_dj2 = ( cross(cross(g2(k,:),j2),j2) ) / ( norm(cross(g2(k,:),j2)) );

        % Jacobian, 1x6
        dej_dx = [dej_dj1, dej_dj2];

        % Update solution using pseudoinverse
        x = x - RecToSphere(lsqminnorm(dej_dx,ej(k,:)));

        % Enumerate loop counter
        %k = k + 1;
        if (ej(k,1) || ej(k,2) || ej(k,3)) <= tol
            break
        end
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

    % Update loop for offset axis estimation
    for k = 1:1000
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
        deo_dx = [deo_do1,deo_do2];

        % Update solution using psuedoinverse
        y = y - RecToSphere(lsqminnorm(deo_dx,eo(k,:)));

        % Enumerate loop counter
        %k = k + 1;
        
        if (eo(k,1) || eo(k,2) || eo(k,3)) <= tol
            break
        end
    end
end