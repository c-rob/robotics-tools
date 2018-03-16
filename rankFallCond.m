% returns a symbolic column vector where each row contains the condition to
% be satisfied to not to have a fall of rank in the J matrix;

function conds=rankFallCond(J)
    r=double(rank(J));
    
    % all minors of order r
    M=minors(J,r);
    
    n=size(M);
    conds=sym('c',n);
    nConds=1;
    for i=1:n
        minor=M{i};
        if det(minor)~=0
            conds(nConds)=det(minor);
            nConds=nConds+1;
        end
    end
    conds=conds(1:nConds-1);
    conds=simplify(conds);
end