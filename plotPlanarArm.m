% What: visualizes at sceen the position of a n- revolute planar arm of
% given the length of the links a and current configuration q0
% Param: dh: DH table with only numeric values in the form
%       [a_i, 0, 0, q0_i]
%       f: optional parameter the haldle of the window in which the
%       function should draw

function plotPlanarArm(dh, f)

if ~isnumeric(dh)
    error('dh must be numerical')
end

% position of the n joints
n=size(dh,1);
p=zeros([n+1,2]);
p(1,:)=[0,0];
T=eye(4);

for i=1:n
    T=T*jointTm(dh(i,:));
    p(i+1,:)=T(1:2,4)';
end

% plot
if nargin==1
    f=figure;
end
figure(f)
hold on
axis equal
plot(p(1:n,1),p(1:n,2),'ko');
plot(p(:,1),p(:,2),'k-');
grid on

% end effector draw
efSize=(dh(:,1)'*ones([n 1]))/20;
v1=(p(n+1,:)-p(n,:))';
v1=v1./norm(v1).*efSize;
v2=[v1(2); -v1(1)];
p_ef=p(n+1,:)'-v2./2;
pts_ef=[p_ef+v1, p_ef, p_ef+v2, p_ef+v1+v2];
plot(pts_ef(1,:), pts_ef(2,:), 'k-');

end

