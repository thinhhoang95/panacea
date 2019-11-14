%% PIN-HOLE CAMERA MODEL PROJECTION
% Input: focal length, aircraft attitude (B-I), aircraft position, landmark
% position
% Output: landmark position in the image plane
% Output (-1, -1) if the landmark falls outside of the image plane

function y = Project(f,R,x,xl)
x(3) = -x(3);
xl(3) = -xl(3);
z = R'*(xl-x)';
if (z(3)==0)
    z(3)=1;
end
ze = [f*z(1)/z(3), f*z(2)/z(3)];
if (ze(1)>0.019 || ze(2)>0.019)
    y = [-1 -1];
else
    y = ze;
end
end