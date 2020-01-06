%% PANACEA IMU PROPAGATION
% Run everytime an IMU value is received

function [Xay,Pay] = PanaceaIMU(Xa,Pa,Q,Aa,Ba,u)
% State propagation and correction
Xay = Aa * Xa + Ba * u;
% Covariance propagation
Pay = Aa * Pa * Aa' + Ba * Q * Ba';
end