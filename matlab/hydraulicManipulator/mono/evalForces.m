% Function that evaluates the generalized applied forces

% Input
%  p:       Hydraulic pressures
%  qd:      Generalized velocities of the system
%  SYS:     Structure with system information
%   .A:         Piston area (m^2)
%   .c:         Viscous friction coefficient in actuator (Ns/m)
%   .g:         Gravity (m/s^2)
%   .m:         First rod mass, distributed (kg)
%   .mh:        Point mass at the end of second rod (kg)
%   .mp:        Point mass at the end of first rod (kg)

% Output
%  Q:      Generalized forces term (7 x 1)

function Q = evalForces(p, qd, SYS)

% Retrieve variables
m   = SYS.m;
g   = SYS.g;
mp  = SYS.mp;
mh  = SYS.mh;
A   = SYS.A;
c   = SYS.c;
sd  = qd(7);
p1  = p(1);
p2  = p(2);

% Update
Q = [0.0; -m*g; 0.0; -mp*g; 0.0; -mh*g; (p2-p1)*A - c*sd];

end