% what: this function composes the B(q) inertia matrix of the robot used in
%       the lagrange model. Inertia of rotors is not modelled.
%       MOVING FRAMES ALGORYTHM
% input: dh: (n x 4) symbolic matrix containing the joints parameters. 
%           Each row must contain one qi variable
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
% returns: B (n x n) matrix symbolic at least in 'qi' components

function B=inertiaMatrix2(dh, m, I, d, ~)

% init
n=size(dh,1);
z0=[0;0;1];
[~,jTNum]=jointType(dh);
dq=sym('dq',[n,1]);
dq=sym(dq,'real');


omegaPrev=[0;0;0];
velPrev=[0;0;0];
T=0;
for i=1:n   % i-th link
    
    % rotation to the next frame
    R=jointTm(dh(i,:));
    rO=R(1:3,4);
    R=R(1:3,1:3);
    
    % velocities
    omega=R'*(omegaPrev + jTNum(i)*dq(i)*z0);
    vel=R'*(velPrev + (1-jTNum(i))*dq(i)*z0 + cross(omega,R'*rO));
    velC=vel+cross(omega,-R'*rO*d(i));
    
    % kinetic energy
    Ti=(m(i)*(velC'*velC)+omega'*diag(I(i,:))*omega)/2;
    T=T+Ti;
    
    % print?
    if nargin==5
        omega=simplify(omega)
        vel=simplify(vel)
        velC=simplify(velC)
    end
    
    % next step
    omegaPrev=omega;
    velPrev=vel;
   
end

% quadratic form
B=jacobian(jacobian(T,dq),dq);

B=simplify(B);

end