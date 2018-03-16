% given n in the workspace it defines the variables needed for dynamic
% analysis

% state variables
q=sym('q',[n,1]);
q=sym(q,'real');

% velocities
dq=sym('dq',[n,1]);
dq=sym(dq,'real');

% accelerations
ddq=sym('ddq',[n,1]);
ddq=sym(ddq,'real');

% distances of the centres of mass
d=sym('d',[n,1]);
d=sym(d,'real');

% link lenght
l=sym('l',[n,1]);
l=sym(l,'real');

% link masses
m=sym('m',[n,1]);
m=sym(m,'real');

% inertia matrices
I=sym('I',[n,3]);
I=sym(I,'real');

syms t real