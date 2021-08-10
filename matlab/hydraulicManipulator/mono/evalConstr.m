% Function that evaluates the kinematic constraint equations of the manipulator

% Input
%  q:       Generalized coordinates of the system
%  SYSTEM:  Structure with system information
%   .L          Length of first rod of the manipulator
%   .xA         x coordinate of fixed point A
%   .yA         y coordinate of fixed point A
%   .xB         x coordinate of fixed point B
%   .yB         y coordinate of fixed point B

% Output
%  Phi:     Array of kinematic constraints (5 x 1)

function Phi = evalConstr(q, SYSTEM)

% Preallocate
Phi = zeros(5,1);

% Extract variables
xA = SYSTEM.xA;
yA = SYSTEM.yA;
xB = SYSTEM.xB;
yB = SYSTEM.yB;
L  = SYSTEM.L;

x1 = q(1);
y1 = q(2);
x2 = q(3);
y2 = q(4);
x3 = q(5);
y3 = q(6);
s  = q(7);

Phi(1) = (x1-xA)^2 + (y1-yA)^2 - (0.5*L)^2;
Phi(2) = (x2-xA) - 2.0*(x1-xA);
Phi(3) = (y2-yA) - 2.0*(y1-yA);
Phi(4) = (x3-x2)^2 + (y3-y2)^2 - SYSTEM.L23^2;
Phi(5) = (x1-xB)^2 + (y1-yB)^2 - s^2;

end