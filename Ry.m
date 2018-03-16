function R=Ry(a)

R=[cos(a),0,sin(a);0,1,0;-sin(a),0,cos(a)];

if isempty(coder.target) && isfloat(R)
    R=roundn(R,-15);
end

end