A = [1 0; 1 0];
B = [1; 0];
C = [1 -1; 0 1];
D = 0;

Q = 0.5;
R = 0.2;
% u = Q*randn([1 100]);
% w = R*randn([1 100]);
X = zeros(2,length(u));
P = zeros(2,2,length(u));

for k=2:length(u)
    if (k>3)
        z = zeros(2,1);
        X_minus = A*X(:,k) + B*u(k);
        P_minus = A*P(:,:,k)*A' + B*Q*B';
        z(1) = u(k);
        z(2) = X_minus(2) + w(k);
        S = C*P_minus*C' + [Q; P_minus(2,2)];
        K = P_minus*C'*inv(S);
        X(:,k+1) = X(:,k) + K*(z-C*X_minus);
        P(:,:,k+1) = (eye(2) - K*C)*P_minus;
    else
        X(:,k+1) = A*X(:,k) + B*u(k);
        P(:,:,k+1) = A*P(:,:,k)*A' + B*Q*B';
    end
end