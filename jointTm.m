% What: composes the homogeneous transformation associated to the 4 DH
%       parameters
% Param: dhI=[a, alpha, d, theta] DH parameters
% Return: T transformation matrix

function T = jointTm(dhI)

T1=Tm(Rz(dhI(4)), [0;0;dhI(3)]);
T2=Tm(Rx(dhI(2)), [dhI(1);0;0]);
T=T1*T2;

end

