% what: computes the overall homogeneous transformation matrix of the n
% joints as given in the dh table in input.
% param: dh: (n x 4) DH table
% return: homogeneous transf matrix

function T=jointsTm(dh)

n=size(dh,1);
T=eye(4);
for i=1:n
    T=T*jointTm(dh(i,:));
end

if ~isnumeric(T)
    T=simplify(T);
end
end