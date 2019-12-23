clc;

N_window = 5; % Window length
%N_window+2 x 6 is the total matrix size

% Initialize the augmented A matrix
Aa = [Add zeros(6,(N_window+1)*6)];
Aa_row = [eye((N_window+1)*6) zeros((N_window+1)*6,6)];
Aa = [Aa; Aa_row];
clear Aa_row;

% Initialize the augmented B matrix
Ba = [Bdd; zeros((N_window+1)*6,3)];

Q = 0.05*eye(3);
P = zeros((N_window+2)*6);
u = 0.05 * ones(6,6);

