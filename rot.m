% What: this function compose the rotation matrix equivalent to the 3 successive
% rotation angles about axes. 
% Params:   angles: array of dimension n in radiants
%           axes: string of leght n made of 'x', 'y', 'z'
% Return:   R: rotation matrix as result

function R=rot(angles, axes)

if isrow(angles)
    angles=angles';
end

[n,~]=size(angles);

R=eye(3);

for i=1:n
    switch axes(i)
        case 'x'
            R=R*Rx(angles(i));
        case 'y'
            R=R*Ry(angles(i));
        case 'z'
            R=R*Rz(angles(i));
        otherwise
            error('bad formatted axes');
    end
end