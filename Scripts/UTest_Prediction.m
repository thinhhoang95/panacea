clc;

N_window = 23; % Window length
%N_window+2 x 6 is the total matrix size

% Initialize the augmented A matrix
Aa = [Add zeros(6,(N_window+1)*6)];
Aa_row = [eye((N_window+1)*6) zeros((N_window+1)*6,6)];
Aa = [Aa; Aa_row];
clear Aa_row;

% Initialize the augmented B matrix
Ba = [Bdd; zeros((N_window+1)*6,3)];

% Uncertainty of the input
Q = 0.05*eye(3);
% Uncertainty of the measurement
R = 0.001*eye(numOfLm * 2);
% Intial state
X = zeros((N_window+2)*6,1);
% Initial covariance matrix
P = zeros((N_window+2)*6);
% Input to the system
u = 0.05 * ones(4,3);

state_dest = 4;
for i=1:state_dest
    u_k = u(i,:);
    % State propagation
    X = Aa * X + Ba * u_k';
    P = Aa * P * Aa' + Ba * Q * Ba';
    fprintf('Propagation #: %d \n', i);
    fprintf('State: \n');
    disp(X);
    fprintf('Covariance matrix: \n');
    disp(P);
end