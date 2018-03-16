% Optimized version of the newtonEuler method. NO SYMBOLIC ARGUMENT ALLOWED
% what: this function return the torques at the joints evaluated with the
%       Newton-Euler recursive method. NE(q,dq,ddq). cost O(n).
%       The result is the same as B(q)*ddq+C(q,dq)*dq+g(q) but with a more
%       efficient algorythm.
% input: All numeric values ONLY!
%        dhq: dh-table evaluated at current configuration q
%        dq: current velocity
%        ddq: current acceleration
%        jointT: string of length n containing 'r' and 'p' or
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
% output: torques: (n x 1) numeric joints torques vector

function torques=newtonEulerFast(dhq, dq, ddq, jointT, m, I, d, g0, fe)

% initializations: first place occupied by kink 0
n=size(dhq,1);
z0=[0;0;1];

% initializations: buinding the positions of the centres of mass in local
% frame
rC=zeros(3,n);
for i=1:n
    T=jointTm([-d(i)*dhq(i,1), dhq(i,2), -d(i)*dhq(i,3), dhq(i,4)]);
    rC(:,i)=T(1:3,1:3)'*T(1:3,4);
end

% forward recursion: finding accelerations (all quantities expressed in
%                                           local frames)

% cycle initializations
omega=zeros(3,n+1);
dOmega=zeros(3,n+1);
ddp=zeros(3,n+1);
ddp(:,1)=-g0;
ddPc=zeros(3,n);

for i=1:n
    j=i+1;    % j in the next element, while i is the previous
    
    % elements needed
    R=jointTm(dhq(i,:));
    r=R(1:3,1:3)'*R(1:3,4); % position of the frame i wrt i-1 expressed in i
    R=R(1:3,1:3)';          % rotation of RF i-1 to RF i
    
    % main steps: ang vel, angular and linear accelerations
    if jointT(i)=='p' || jointT(i)==0
        
        omega(:,j)=R*omega(:,i);
        dOmega(:,j)=R*dOmega(:,i);
        ddp1=R*(ddp(:,i)+ddq(i)*z0);
        ddp2=2*dq(i)*cross(omega(:,j),R*z0);
        ddp3=cross(dOmega(:,j),r);
        ddp4=cross(omega(:,j),cross(omega(:,j),r));
        ddp(:,j)=ddp1+ddp2+ddp3+ddp4;
        
    elseif jointT(i)=='r' || jointT(i)==1
        
        omega(:,j)=R*(omega(:,i)+dq(i)*z0);
        dOmega1=dOmega(:,i)+ddq(i)*z0;
        dOmega2=dq(i)*cross(omega(:,i),z0);
        dOmega(:,j)=R*(dOmega1+dOmega2);
        ddp1=R*ddp(:,i);
        ddp2=cross(dOmega(:,j),r);
        ddp3=cross(omega(:,j),cross(omega(:,j),r));
        ddp(:,j)=ddp1+ddp2+ddp3;
    end
    
    ddPc1=ddp(:,j) + cross(dOmega(:,j),rC(:,i));
    ddPc2=cross(omega(:,j),cross(omega(:,j),rC(:,i)));
    ddPc(:,i)=ddPc1+ddPc2;
    
end


% backward recursion: finding joint torques

% cycle initializations
forceOld=fe(1:3);
momtOld=fe(4:6);
torques=zeros(n,1);
RNext=eye(3);

for i=n:-1:1
    j=i+1;    
    
    % rotation of RF i-1 to RF i
    RPrevI=jointTm(dhq(i,:));
    r=RPrevI(1:3,1:3)'*RPrevI(1:3,4); % position of the frame i wrt i-1 expressed in i
    RPrevI=RPrevI(1:3,1:3)';          % rotation of RF i-1 to RF i
        
    
    % main steps: forces, momenta and joint torques.
    force=RNext*forceOld + m(i)*ddPc(:,i);
    m1=-cross(force,r+rC(:,i));
    m2=RNext*momtOld;
    m3=cross(RNext*forceOld,rC(:,i));
    m4=diag(I(i,:))*dOmega(:,j);
    m5=cross(omega(:,j), diag(I(i,:))*omega(:,j));
    momt=m1+m2+m3+m4+m5;
    
    
    if jointT(i)=='p' || jointT(i)==0
        torques(i)=force'*RPrevI*z0;
    elseif jointT(i)=='r' || jointT(i)==1
        torques(i)=momt'*RPrevI*z0;
    end
    
    RNext=RPrevI';
    forceOld=force;
    momtOld=momt;
    
end

end
