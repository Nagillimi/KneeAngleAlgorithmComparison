% Find start of gait cycle
function ZKA_i = ZKA(gShankFE,numCycles)
% numCycles = 8;
% gShankFE = g2_JCS(3,:);
    % Init
    startGait_i = zeros(1,numCycles);
    firstMin_i = zeros(1,numCycles);
    ZKA_i = zeros(1,numCycles);
    max_i = zeros(1,numCycles);
    HS_i = zeros(1,numCycles);
%     probing_length = 1.25*f; % 1 second (factor of freq)
    
    % First index
    i = 2; 

    for j = 1:numCycles
        % Find the start of the gait cycle
        startGait_i(j) = zero_cross_descend(gShankFE,i,-0.05);

        % Find first minimum while descending
        firstMin_i(j) = descend_index(gShankFE,startGait_i(j),-0.05);

        % Find second minimum while descending
        ZKA_i(j) = descend_index(gShankFE,firstMin_i(j),-0.05);

        % Find maximum gyro (+ve) shank rotation. Must be above 30deg/s
        max_i(j) = max_index(gShankFE,ZKA_i(j),30);
        
        % Find third minimum while descending -- this version ignores small
        % humps and probes for deeper values within 100 points
%         thirdMin_i(j) = robust_descend_index(gShankFE,ZKA_i(j),-0.05,probing_length);

        % Find fourth minimum while descending
        HS_i(j) = descend_index(gShankFE,max_i(j),-0.05);
        
        % Iterate index i
        i = HS_i(j);
    end
end

% Debugging
% plot(knee_angle_Allseits)
% hold on
% plot(gShankFE)




% Helper functions
function index = zero_cross_descend(vector,start_i,min_slope_threshold)
    k = start_i;
    
    % Seek for a descent of slope > slope threshold (-ve) to start
    while (vector(k) - vector(k-1)) > min_slope_threshold
        k = k+1; 
    end

    % Seek for the index that crosses zero
    while (vector(k) > 0)
        k = k+1; 
    end
    
    index = k - 1;
end

function index = descend_index(vector,start_i,min_slope_threshold)
    k = start_i;
    
    % Seek for a descent of slope < min slope threshold to start
    while (vector(k) - vector(k-1)) > min_slope_threshold
        k = k+1; 
    end

    % Seek for the index that begins ascending
    while (vector(k) < vector(k-1))
        k = k+1; 
    end
    
    index = k - 1;
end

function index = robust_descend_index(vector,start_i,min_slope_threshold,probing_length)
    k = start_i;
    
    % Seek for a descent of slope < min slope threshold to start
    while (vector(k) - vector(k-1)) > min_slope_threshold
        k = k+1; 
    end

    % Seek for the index that begins ascending AND has a value greater than
    % the min some time after it (probing length)
    while (vector(k) < vector(k-1))
        k = k+1; 
        % Hop past the hump if it exists (hump size resistance = probing l)
        if (vector(k) > vector(k + probing_length))
            k = k + probing_length;
        end
    end
    
    index = k - 1;
end

function index = max_index(vector,start_i,max_val_threshold)
    k = start_i;
    
    % Seek for a val > min slope threshold to start
    while (vector(k) < max_val_threshold)
        k = k+1; 
    end

    % Seek for the index that begins descending
    while (vector(k) > vector(k-1))
        k = k+1; 
    end
    
    index = k - 1;
end
