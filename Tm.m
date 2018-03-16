% composes the homogeneous transformation matrix of given rotation and
% translation

function T=Tm(R,v)
    if isfloat(R) && isfloat(v)
        T=zeros(4,4);
        T(1:3,1:3)=R;
        T(4,4)=1;
        T(1:3,4)=v;
    else
        T=sym('T',[4,4]);
        T(1:3,1:3)=R;
        T(4,4)=1;
        T(4,1:3)=0;
        T(1:3,4)=v;
    end
end