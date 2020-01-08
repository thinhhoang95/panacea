%% PANACEA OPTICAL FLOW CORRECTION
% Input: state at delay i and j (i is "more ancient" than j chronically, thus i>j), res:
% residual optical flow state vector, Xa: augmented state, Pa: augmented
% covariance matrix, Q: covariance of input uncertainty (acceleration), 
% R: covariance of measurement uncertainty (optical flow).
% Output: Xay: output state, Pay: output covariance matrix

function [Xay,Pay,debug] = PanaceaOF(i,j,xev,ofd,psi_i,psi_f,Xa,Pa,R,f,N_window,Aa,u,Ba,Q,clock,stateNum)
numOfLm = length(ofd);

% Determine the "true" optical flow, that is discarding optical flow of
% zero out of the vector =>

coder.varsize('trueOfdIndex', [1, numOfLm]);
trueOfdIndex = [];
for z=1:numOfLm
    if (ofd(z,1)==0) && (ofd(z,2)==0)
        % pass
    else
        trueOfdIndex = [trueOfdIndex z];
    end
end

trueOfd = zeros(length(trueOfdIndex),2);
truexe = zeros(1,length(trueOfdIndex));
trueye = zeros(1,length(trueOfdIndex));
for z=1:length(trueOfdIndex)
    trueOfd(z,:) = ofd(trueOfdIndex(z),:);
    truexe(z) = xev(z,1);
    trueye(z)= xev(z,2);
end

% == Calculate the Kalman gain ==
% psi_f = psiv(j,:);
% psi_i = psiv(i,:);
% x_f = Xa((6*j+1):(6*(j+1)),:);
x_iv = Xa((6*i+1):(6*(i+1)),:);
% -> Initialize variables <-
numOfTrueLm = length(trueOfdIndex);
Hk = zeros(numOfLm*2,(N_window+2)*6);
meas = zeros(numOfLm*2,1);
Rtrue = zeros(numOfLm*2);
% -> Calculate Matrix Hk <-
for k=1:length(trueOfdIndex)
    xe = truexe(k);
    ye = trueye(k);
    dx = trueOfd(k,1);
    dy = trueOfd(k,2);
    % OF Virtual measurement operator
    P = [-f 0 xe+dx; 0 -f ye+dy];
    % Counter rotation operator for target yaw angle
    Y_f = [cos(psi_f) -sin(psi_f) 0; sin(psi_f) cos(psi_f) 0; 0 0 1];
    % Counter rotation operator for initial yaw angle
    Y_i = [cos(psi_i) -sin(psi_i) 0; sin(psi_i) cos(psi_i) 0; 0 0 1];
    % Selection of position states (x,y,z) out of 6 members state
    % (x,y,z,vx,vy,vz)
    Sf1 = [eye(3) zeros(3)];
    % Selection of the jth state out of the window
    Sf2 = zeros(6,(N_window+2)*6);
    Sf2(:,(6*j+1):(6*(j+1))) = eye(6);
    % Append to the virtual measurement matrix
    Hk((k-1)*2+1:k*2,:) = P*Y_f*Sf1*Sf2;
    % Trim the R matrix to only match available measurements
    Rtrue(k,:) = R(trueOfdIndex(k),:);
    % -> Calculate the measurement vector <-
    % -> from the original state <- %
    x_i = Sf1*x_iv;
    % meas is from projected 
    meas((k-1)*2+1:k*2,:) = -P*Y_f*(-[x_i(1); x_i(2); 0] + x_i(3)/f*inv(Y_i)*[xe; ye; 0]);
end

% If stateCount is 0, then it matches the trackingTs, perform Kalman
% correction; otherwise just propagate the state forward!
if (stateNum~=0)
    Xay = Aa * Xa + Ba * u;
    Pay = Aa * Pa * Aa' + Ba * Q * Ba';
else
    Lk = Aa * Pa * Hk' / (Hk * Pa * Hk' + R);
    % Lk = zeros(42,100);
    Xay = Aa * Xa + Ba * u + Lk * (meas - Hk * Xa);
    Pay = Aa * Pa * (Aa - Lk * Hk)' + Ba * Q * Ba';
    % Xay = Aa * Xa + Ba * u;
    % Pay = Aa * Pa * Aa' + Ba * Q * Ba';
    fprintf('Kalman filter is executed at time: ');
    disp(clock);
end

% Exporting data to the debug port
debug = [Pay(1,1) Pay(2,2) Pay(3,3)];
end