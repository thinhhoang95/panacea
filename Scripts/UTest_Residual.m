psi_i = 0.12;
psi_f = 0.06;
R_i = angle2dcm(psi_i,0,0); % B-I
R_f = angle2dcm(psi_f,0,0); % B-I
x_i = [1 0.5 -5.6];
x_f = [1.4 0.3 -6];

xe_i = Project(f,R_i,x_i,[0 0 0]);
xe_f = Project(f,R_f,x_f,[0 0 0]);

fprintf('xe_i: ');
disp(xe_i);
fprintf('xe_f: ');
disp(xe_f);

ofd = xe_f - xe_i;
fprintf('ofd: ');
disp(ofd);

% Backprojection of ancient state
xl_i = -x_i(3)/f* R_i *[xe_i -f]' + x_i';
fprintf('Backprojected landmark position: \n');
disp(xl_i);

r = Measurement(f,xe_i(1),xe_i(2),ofd(1),ofd(2),psi_i,psi_f,x_i',x_f');
fprintf('Residue is: \n');
disp(r);