%% PANACEA OPTICAL FLOW CORRECTION
% Input: state at delay i and j (i is before j chronically, thus i>j), res:
% residual optical flow state vector, Xa: augmented state, Pa: augmented
% covariance matrix, Q: covariance of input uncertainty (acceleration), 
% R: covariance of measurement uncertainty (optical flow).
% Output: Xay: output state, Pay: output covariance matrix

function [Xay,Pay] = PanaceaOF(i,j,res,Xa,Pa,Q,R)
numOfLm = length(ofd);

% 
