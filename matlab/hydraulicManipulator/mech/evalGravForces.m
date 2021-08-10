% Evaluates the gravitational forces of the mechanical system

% Input
%  th:      Rod angles w.r.t. x axis
%  MECH:    Structure with system parameters
%   .L:         Length of first rod (m)
%   .L23:       Length of second rod (m)
%   .m:         First rod mass, distributed (kg)
%   .mh:        Point mass at the end of second rod (kg)
%   .mp:        Point mass at the end of first rod (kg)

% Ouput
%  fg:      Gravitational forces term (2 x 1)

function fg = evalGravForces(th, MECH)

% Retrieve angles
th1 = th(1);
th2 = th(2);

% Evaluate
fg(1,1) = -cos(th1) * MECH.L * MECH.g * (0.5*MECH.m + MECH.mp + MECH.mh);
fg(2,1) = - MECH.mh * MECH.L23 * MECH.g * cos(th2);

end