% what: this function returns the 'g' vector symbolic in q as appear in the
%       Lagrange dynamic model.
% input: dh: (n x 4) symbolic matrix containing the joints parameters. 
%           Each row must contain one q_i variable
%        g0: 3-dimensional gravity vector in the base frame
%        MLinks: n dimensional vector containing the total masses of the
%           links
%        PCentres: n dimensional vector of the distances of the centes of
%           mass of the links along the structure w.r.t the following
%           frame. distances along the link normalized in [0-1]
%           Simplifying assumption of rectilinear links.
% return: g: (n x 1) vector symbolic in q

function g=gravityTerm(dh, g0, MLinks, PCentres)

n=size(dh,1);
g=sym('g',[n,1]);
J=cell([n,1]);          % J{i}(x,j) position jacobian component of the
                        %   x-coordinate due to a velocity in j-coordinate
                        %   of the i-th centre of mass

% obtaining jacobians of the centre of masses
for i=1:n   % for each link
    
   % position of the i-th centre of mass
    dhT=dh(1:i,:);
    
    dhT(i)=[dhT(i,1)-PCentres(i)*dhT(i,1), dhT(i,2), ...
        dhT(i,3)-PCentres(i)*dhT(i,3), dhT(i,4)];
    
    % jacobian of the link
    Ji=gjacob(dhT);
    J{i}=[Ji(1:3,:), zeros(3,n-i)];
end
    
% composing g vectors
for i=1:n   % for each link
    g(i)=0;
    for j=1:n   % for each joint
        g(i)=g(i)-MLinks(j)*g0'*J{j}(:,i);
        g(i)=simplify(g(i));
    end
end

g=simplify(g);

end