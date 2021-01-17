% Find start of gait cycle
function ZKA_i = ZKA(gShankFE,numCycles)
% numCycles = 8;
% gShankFE = g2_JCS(3,:);
    % Init
    startGait_i = zeros(1,numCycles);
    firstMin_i = zeros(1,numCycles);
    ZKA_i = zeros(1,numCycles);
    thirdMin_i = zeros(1,numCycles);
    HS_i = zeros(1,numCycles);

    % First index
    i = 2;

    for j = 1:numCycles
        % Find the start of the gait cycle
        startGait_i(j) = zero_cross_descend(gShankFE,i,-0.04);

        % Find first minimum while descending
        firstMin_i(j) = descend_index(gShankFE,startGait_i(j),-0.04);

        % Find second minimum while descending
        ZKA_i(j) = descend_index(gShankFE,firstMin_i(j),-0.04);

        % Find third minimum while descending
        thirdMin_i(j) = descend_index(gShankFE,ZKA_i(j),-0.04);

        % Find fourth minimum while descending
        HS_i(j) = descend_index(gShankFE,thirdMin_i(j),-0.04);
        
        % Iterate index i
        i = HS_i(j);
    end
end

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
