%% IMUStatePredict
% Input: old_state comprises of (x,y,z,vx,vy,vz), covariance, acceleration
% including (ax,ay,az) as well as covariance. Output new_state with
% new_covariance.

function [new_state, new_cov] = IMUStatePredict(old_state, old_cov, accel, accel_cov, Add, Bdd)
    new_state = Add * old_state + Bdd * accel;
    new_cov = Add' * old_cov * Add + accel_cov;
end