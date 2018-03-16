% what: this function returns the symbolic expression alpha contained in
%      the residuals formula computed from the robot model without
%      friction.
% param: B: (n x x) symbolic inertia matrix of the robot
%        g: (n x 1) symbolic gravity term of the model
% return: alpha: (n x 1) symbolic expression (at least) in q, dq

function alpha=alphaResidual(B, g)

% variables vector
n=size(B,1);
alpha=sym('a',[n,1]);
alpha(:,:)=zeros(n,1);
q=sym('q',[n,1]);
q=sym(q,'real');
dq=sym('dq',[n,1]);
dq=sym(dq,'real');

for i=1:n  % for each joint
    for j=1:n  % velocities
        for k=1:n  % velocities
            
            alpha(i)=alpha(i)-diff(B(j,k),q(i))*dq(j)*dq(k)/2;
            
        end
    end
    
    alpha(i)=alpha(i)+g(i);
end

end