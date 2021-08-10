% Evaluates the mass matrix of the mechanical system

% Input
%  th:      Rod angles w.r.t. x axis
%  MECH:    Structure with system parameters
%   .L:         Length of first rod (m)
%   .L23:       Length of second rod (m)
%   .m:         First rod mass, distributed (kg)
%   .mh:        Point mass at the end of second rod (kg)
%   .mp:        Point mass at the end of first rod (kg)

% Ouput
%  MM:      Mass matrix (2 x 2)

function MM = evalMassMatrix(th, MECH)

% Retrieve angles
th1 = th(1);
th2 = th(2);

% Terms in mass matrix
mth1    = MECH.L^2 * (1.0/3.0 * MECH.m + MECH.mp + MECH.mh);
mth2    = MECH.L23^2 * MECH.mh;
mcp     = MECH.L * MECH.L23 * cos(th1-th2) * MECH.mh;

MM = [mth1, mcp; mcp, mth2];

end