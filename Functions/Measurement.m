%% CALCULATE THE LANDMARK MEASUREMENT RESIDUAL
% The form is linear suitable for LTV Kalman Filter
% Input: f: focal length, xe, ye: coordinate in img plane, dx, dy: optical
% flow, psi_i: initial yaw angle (I-B), psi_f: target yaw angle, x_i: initial
% position, x_f: target position

function residual = Measurement(f, xe, ye, dx, dy, psi_i, psi_f, x_i, x_f)
    P = [-f 0 xe+dx; 0 -f ye+dy];
    Y_f = [cos(psi_f) -sin(psi_f) 0; sin(psi_f) cos(psi_f) 0; 0 0 1];
    Y_i = [cos(psi_i) -sin(psi_i) 0; sin(psi_i) cos(psi_i) 0; 0 0 1];
    residual = -P*Y_f*x_f - P*Y_f*(-[x_i(1); x_i(2); 0] + x_i(3)/f*inv(Y_i)*[xe; ye; 0]);
end