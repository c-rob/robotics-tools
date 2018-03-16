% what: this function returns the matrix C factorization where Coriolis 
%       forces and Centrifugal forces appear explicitely; as appear in the
%       dynamic model C(q,dq)*dq
% Input: B: (n x n) inertia matrix symbolic in q
% return: C: (n x n) symbolic in q, dq

function C=centrCoriolHTerm(B)

% variables vector
n=size(B,1);
q=sym('q',[n,1]);
q=sym(q,'real');
dq=sym('dq',[n,1]);
dq=sym(dq,'real');

% output
C=sym('c',[n,n]);

Hjk=sym('c',[n,n])*zeros(n,n);     % temporary matrix

for i=1:n
    for j=1:n
        for k=1:n   % symmetric matrix in j k
            Hjk(j,k)=diff(B(i,j),q(k))-diff(B(j,k),q(i))/2;
        end
    end
    
    C(i,:)=dq'*Hjk;
end

C=simplify(C);

end