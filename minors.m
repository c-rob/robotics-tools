% returns a cell array containing the minors of the A matrix of order k

function M=minors(A,k)
    [m,n]=size(A);
    
    % indexes
    rowsInd=combnk(1:m,k);
    colsInd=combnk(1:n,k);
    [nRowsInd,~]=size(rowsInd);
    [nColsInd,~]=size(colsInd);
    
    % creating matrices
    M=cell(nRowsInd*nColsInd,1);
    for i=1:nRowsInd
        for j=1:nColsInd
            M{nColsInd*(i-1)+j}=A(rowsInd(i,:),colsInd(j,:));
        end
    end
end