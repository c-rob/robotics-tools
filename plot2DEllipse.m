% what: this function plot a 2D ellipse in the plane given the matrix
%       representation of its quadratic form
% input: C: (2 x 2) numeric matrix of coefficients of the quadratic form
%        x0: (2 x 1) centre of the ellipse
%        f: figure handle where the function should plot (optional)

function plot2DEllipse(C, x0, f)

if nargin==2
    f=figure;
end


% control of the C input
if C(1,2)~=C(2,1)
    error('The input C does not represent a coninc')
elseif det(C)<0
    error('The conic C is not an ellipse');
end

n=50; % 50 points of interpolation

[V,D]=eig(C);
T=linspace(0,2*pi,n);
D=diag(1./sqrt(diag(D)));

X=[cos(T); sin(T)];
X=(V*D)*X;
X=[X(1,:)+x0(1); X(2,:)+x0(2)];

figure(f)
hold on
plot(X(1,:),X(2,:),'b');

axis equal

end