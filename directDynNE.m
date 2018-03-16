% what: this function computes the direct dynamics for robot simulation
%      given its dynamic parameters. joint's acceleration is computed given
%      torques and actual system configuration. NewtonEulerFast is used to
%      solve the problem in a fast, general way. 
%      ddq=inv(B)(q)*(torq-(J'*fe+c(q,dq)*dq+g(q)))
% input: All numeric values ONLY!
%        dhq: (n x 4) dh-table evaluated at current configuration q
%        dq: (n x 1) current velocity
%        torq: (n x 1) joint torques
%        jT: string of length n containing 'r' and 'p' or
%           array of 1s for revolute and 0s for prismatic.
%        m: n dimensional vector containing the total masses of the
%           links
%        I: (n x 3) inertia constants matrices diagonal of link i.
%           The 3 inertias of link i are in the row i.
%           The diagonal property of the matrices is an assumpion of
%           siplicity sinche the script is use for analisys.
%        d: n dimensional vector of the distances of the centes of
%           mass of the links along the structure w.r.t the following
%           frame. distances along the link normalized in [0-1]
%           Simplifying assumption of rectilinear links.
%        g0: gravity vector in the 0 frame.
%        fe: (6 x 1) end effector force and moment at the tip.
% return: ddq: (n x 1) joint accelerations

function ddq=directDynNE(dhq, dq, torq, jT, m, I, d, g0, fe)

n=size(dhq,1);
zerN=zeros(n,1);
zer6=[0;0;0;0;0;0];
zer3=[0;0;0];
B=zeros(n,n);

% NE(q,dq,ddq=0,fe,g)=J'*fe+c(q,dq)*dq+g(q)
mixedTorq = newtonEulerFast(dhq, dq, zerN, jT, m, I, d, g0, fe);

% NE(q,dq=0,e_i,fe=0,g=0)=b_i    i-th column of B
for i=1:n
    e_i=zerN;
    e_i(i)=1;
    B(:,i) = newtonEulerFast(dhq, zerN, e_i, jT, m, I, d, zer3, zer6);
end

ddq=B\(torq-mixedTorq);

end