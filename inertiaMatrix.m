% what: this function composes the B(q) inertia matrix of the robot used in
%       the lagrange model. Inertia of rotors is not modelled.
% input: dh: (n x 4) symbolic matrix containing the joints parameters. 
%           Each row must contain one qi variable
%        MLinks: n dimensional vector containing the total masses of the
%           links
%        ILinks: (n x 3) inertia constants matrices diagonal of link i.
%           The 3 inertias of link i are in the row i.
%           The diagonal property of the matrices is an assumpion of
%           siplicity sinche the script is use for analisys.
%        PCentres: n dimensional vector of the distances of the centes of
%           mass of the links along the structure w.r.t the following
%           frame. distances along the link normalized in [0-1]
%           Simplifying assumption of rectilinear links.
% returns: B (n x n) matrix symbolic at least in 'qi' components

function B=inertiaMatrix(dh, MLinks, ILinks, PCentres)

n=size(dh,1);
B=zeros(n,n);

for i=1:n   % i-th link
    
    % position of the i-th centre of mass
    dhT=dh(1:i,:);
    
    dhT(i)=[dhT(i,1)-PCentres(i)*dhT(i,1), dhT(i,2), ...
        dhT(i,3)-PCentres(i)*dhT(i,3), dhT(i,4)];
    
    % jacobian of the link
    Ji=[gjacob(dhT),zeros([6 n-i])];
    
    % coordinate frame change
    R=jointsTm(dhT);
    R=R(1:3,1:3);
    
    % B(q) formula
    Bi=(MLinks(i)*Ji(1:3,:)'*Ji(1:3,:)+ ...
        Ji(4:6,:)'*R*diag(ILinks(i,:))*R'*Ji(4:6,:));
    B=B+Bi;
    
end

B=simplify(B);

end