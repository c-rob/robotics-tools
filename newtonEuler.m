% what: this function return the torques at the joints evaluated with the
%       Newton-Euler recursive method. NE(q,dq,ddq). cost O(n).
%       The result is the same as B(q)*ddq+C(q,dq)*dq+g(q) but in
%       efficien algorythm. The function in intended to work with numeric
%       inputs.
% input: q: current configuration
%        dq: current velocity
%        ddq: current acceleration
%        dh: (n x 4) symbolic matrix containing the joints parameters. 
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
%        g0: gravity vector in the 0 frame.
%        fe: (6 x 1) end effector force and moment at the tip.
% output: torques: (n x 1) numeric joints torques vector

function torques=newtonEuler(q, dq, ddq, dh, m, I, d, g0, fe)

% initializations: first place occupied by kink 0
n=size(dh,1);
jointT=jointType(dh);
dh=double(subs(dh, sym('q',[n,1]), q));
z0=[0;0;1];

% initializations: buinding the positions of the centres of mass in local
% frame
rC=zeros(3,n);
for i=1:n
    T=jointTm([-d(i)*dh(i,1), dh(i,2), -d(i)*dh(i,3), dh(i,4)]);
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
    R=jointTm(dh(i,:));
    r=R(1:3,1:3)'*R(1:3,4); % position of the frame i wrt i-1 expressed in i
    R=R(1:3,1:3)';          % rotation of RF i-1 to RF i
    
    % main steps: ang vel, angular and linear accelerations
    if jointT(i)=='p'
        
        omega(:,j)=R*omega(:,i);
        dOmega(:,j)=R*dOmega(:,i);
        ddp(:,j)=R*(ddp(:,i)+ddq(i)*z0) + 2*dq(i)*cross(omega(:,j),R*z0) +...
            cross(dOmega(:,j),r) + cross(omega(:,j),cross(omega(:,j),r));
        
    elseif jointT(i)=='r'
        
        omega(:,j)=R*(omega(:,i)+dq(i)*z0);
        dOmega(:,j)=R*(dOmega(:,i)+ddq(i)*z0+dq(i)*cross(omega(:,i),z0));
        ddp(:,j)=R*ddp(:,i) + cross(dOmega(:,j),r) + ...
            cross(omega(:,j),cross(omega(:,j),r));
    end
    
    ddPc(:,i)=ddp(:,j) + cross(dOmega(:,j),rC(:,i)) + ...
        cross(omega(:,j),cross(omega(:,j),rC(:,i)));
    
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
    RPrevI=jointTm(dh(i,:));
    r=RPrevI(1:3,1:3)'*RPrevI(1:3,4); % position of the frame i wrt i-1 expressed in i
    RPrevI=RPrevI(1:3,1:3)';          % rotation of RF i-1 to RF i
        
    
    % main steps: forces, momenta and joint torques
    force=RNext*forceOld + m(i)*ddPc(:,i);
    momt=-cross(force,r+rC(:,i)) + RNext*momtOld + ...
        cross(RNext*forceOld,rC(:,i)) + diag(I(i,:))*dOmega(:,j) + ...
        cross(omega(:,j), diag(I(i,:))*omega(:,j));
    
    
    if jointT(i)=='p'
        torques(i)=force'*RPrevI*z0;
    elseif jointT(i)=='r'
        torques(i)=momt'*RPrevI*z0;
    end
    
    RNext=RPrevI';
    forceOld=force;
    momtOld=momt;
end

end