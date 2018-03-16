% formatta per rendere piu leggibili le matrici
% mode:
% 'n': semplice. cos(x)-> C(x)
% 'c': compatta. cos(t1)-> C1
% 's': compatta con semplificazione. [..]->cos(t1 + t2)->C12

function c=shortM(M, mode)
    %default mode='s'
    if nargin==1
        mode='s';
    end
    switch mode
        case 'n'
            a=strrep(char(M),'cos','C');
            c=sym(strrep(a,'sin','S'));
        case {'c','s'}
            if mode=='s'
                M=simplify(M);
            end
                [m, n]=size(M);
                c{m, n}='';
                for i=1:m
                    for j=1:n
                        c{i,j}=regexprep(char(M(i,j)),'cos\(.(\d)(?:\(t\))?\s*\+\s*.(\d)(?:\(t\))?\s*\+\s*.(\d)(?:\(t\))?\s*\+\s*.(\d)(?:\(t\))?\)','C$1$2$3$4'); % cos(t1 + t2 + t3 + t4)
                        c{i,j}=regexprep(c{i,j},'sin\(.(\d)(?:\(t\))?\s*\+\s*.(\d)(?:\(t\))?\s*\+\s*.(\d)(?:\(t\))?\s*\+\s*.(\d)(?:\(t\))?\)','S$1$2$3$4'); % sin(t1 + t2 + t3 + t4)
                        c{i,j}=regexprep(c{i,j},'cos\(.(\d)(?:\(t\))?\s*\+\s*.(\d)(?:\(t\))?\s*\+\s*.(\d)(?:\(t\))?\)','C$1$2$3'); % cos(t1 + t2 + t3)
                        c{i,j}=regexprep(c{i,j},'sin\(.(\d)(?:\(t\))?\s*\+\s*.(\d)(?:\(t\))?\s*\+\s*.(\d)(?:\(t\))?\)','S$1$2$3'); % sin(t1 + t2 + t3)
                        c{i,j}=regexprep(c{i,j},'cos\(.(\d)(?:\(t\))?\s*\+\s*.(\d)(?:\(t\))?\)','C$1$2'); % cos(t1 + t2)
                        c{i,j}=regexprep(c{i,j},'sin\(.(\d)(?:\(t\))?\s*\+\s*.(\d)(?:\(t\))?\)','S$1$2'); % sin(t1 + t2)
                        c{i,j}=regexprep(c{i,j},'cos\(.(\d)(?:\(t\))?\)','C$1');
                        c{i,j}=regexprep(c{i,j},'sin\(.(\d)(?:\(t\))?\)','S$1');
                    end
                end
    end
    
    %facoltativo?
    c=sym(c);
end