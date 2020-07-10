
%physical params
L1 = 0.425; L2 = 0.392;
W1 = 0.109; W2 = 0.082;
H1 = 0.089; H2 = 0.095;

%screw axes
b1=[0  1 0 W1+W2 0 L1+L2]';
b2=[0  0 1 H2 -L1-L2 0]';
b3=[0  0 1 H2 -L2 0]';
b4=[0  0 1 H2 0 0]';
b5=[0 -1 0 -W2 0 0]';
b6=[0  0 1 0 0 0]';
Blist=[b1 b2 b3 b4 b5 b6];

%home configuration
M = [-1 0 0 L1+L2;
      0 0 1 W1+W2;
      0 1 0 H1-H2;
      0 0 0 1];
  
%desired configuration
Tsd=[ 0 1  0 -0.5;
      0 0 -1 0.1;
     -1 0  0 0.1;
      0 0  0 1];
  
%initial guess
thetalist0=[0,-2,-1,-2,3,-1]';

%tolerances
eomg=0.001;
ev=0.0001;

%solve inverse kinematics
[thetalist, success] = IKinBodyIterates(Blist, M, Tsd, thetalist0, eomg, ev);

%approve result by comparing with forward kinematics
T = FKinBody(M,Blist,thetalist)