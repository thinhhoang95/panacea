%% PANACEA Algorithm
% Object detection yet to be implemented
% X_old: old state vector
% action: one of the following: 'imu', 'of', 'od' (not implemented)
% payload: for the action, 'imu': a vector of inertial acceleration
% 'of': a struct consists of: i, j and residual vector
% buffer: the total size of the window X (window of buffer)

function X = Panacea(X_old, action, payload, buffer)
switch(action)
    case 'imu'
        % Propagate the state forward
    case 'of'
        % Handle optical flow data
    case 'od'
        % Handle object detection
    otherwise
        % Do nothing
end
end