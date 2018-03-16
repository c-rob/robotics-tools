function R=Rx(a)

R=[1,0,0;0,cos(a),-sin(a);0,sin(a),cos(a)];

if isempty(coder.target) && isfloat(R)
    R=roundn(R,-15);
end

end