% what: this function creates the geometric Jacobian of the robot described
%       by the DH table.
% input: dh: (n x 4) symbolic matrix containing the joints parameters [a, alpha, d, theta]. 
%           Each row must contain one qi symbolic variable (with i=1,2,..)
%        Tref: optional parameter, 3x3 rotation matrix if the jacobian has
%           to be expressed w.r.t. other fixed coordinates
% return: J the constructed (6 x n) jacobian, simbolic

function J=gjacob(dh, Tref)

% initializations
n=size(dh,1);
T=eye(4); % the current homogeneous transf matrix
J=sym('j',[6,n]); % jacobian
jT=jointType(dh);
Z=sym('z',[3,n+1]); % direction of z axes, starting from z0
P=sym('p',[3,n+1]); % position of axes origin, starting from p0

Z(:,1)=[0;0;1];
P(:,1)=[0;0;0];
for i=1:n       
    T=T*jointTm(dh(i,:));
    T=simplify(T);
   
    Z(:,i+1)=T(1:3,3);
    P(:,i+1)=T(1:3,4);
end


% composition
for i=1:n
    if jT(i)=='p'
        J(:,i)=[Z(:,i);0;0;0];
    elseif jT(i)=='r'
        J(:,i)=[cross(Z(:,i), P(:,n+1)-P(:,i)); Z(:,i)];
    end
end

if ~isnumeric(J)
    J=simplify(J);
end

if nargin==2
    if ~isnumeric(Tref)
        error('Transformation matrix must be numeric.');
    end
    J=[Tref, zeros([3,3]); zeros([3,3]), Tref]*J;
end

end