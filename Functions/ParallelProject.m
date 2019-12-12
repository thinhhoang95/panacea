%% PARALLEL PLANE PROJECTION
% Input: current attitude (B->I)
% Output: a rotation matrix to rotate from body to parallel plane frame

function y = ParallelProject(R)
    e3 = R*[0 0 1]';
    be1 = [1 0 0]';
    e1 = be1 - (be1'*e3)*e3;
    e1 = e1/norm(e1);
    e2 = cross(e3,e1);
    y = [e1 e2 e3];
end