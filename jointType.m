% what: returns a string containing the type of each joint
% param: dh, dh table containing on each row a symbolic variable q1, q2..
% return: jT: string made of 'r' and 'p'
%         jTNum: array on 1s for 'r' and 0s for 'p'

function [jT,jTNum]=jointType(dh)

n=size(dh,1);
jT=blanks(n);
jTNum=zeros(1,n);
q=sym('q',[n,1]);

for i=1:n
    v=dh(i,:);
    if v(3)==q(i) % q=d
        jT(i)='p';
        jTNum(i)=0;
    elseif v(4)==q(i) % q=th
        jT(i)='r';
        jTNum(i)=1;
    else
        error('bad formatted dh table: missing '+q(i));
    end
end

end