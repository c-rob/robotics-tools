function R=Rz(a)

R=[cos(a), -sin(a), 0; sin(a), cos(a), 0; 0, 0, 1];

if isempty(coder.target) && isfloat(R)
    R=roundn(R,-15);
end

end