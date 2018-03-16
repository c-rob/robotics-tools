% what: this function returns the matrix C factorization through the
%       Christoffel symbols as appear in the dynamic model C(q,dq)*dq
% Input: B: (n x n) inertia matrix symbolic in q
% return: C: (n x n) symbolic in q, dq

function C=coriolisTerm(B)

% variables vector
n=size(B,1);
q=sym('q',[n,1]);
q=sym(q,'real');
dq=sym('dq',[n,1]);
dq=sym(dq,'real');

% output
C=sym('c',[n,n]);

Cjk=sym('c',[n,n])*zeros(n,n);     % temporary matrix

for i=1:n
    for j=1:n
        for k=1:n   % symmetric matrix in j k
            Cjk(j,k)=(diff(B(i,j),q(k))+diff(B(i,k),q(j))-diff(B(j,k),q(i)))/2;
        end
    end
    
    C(i,:)=dq'*Cjk;
end

C=simplify(C);

end