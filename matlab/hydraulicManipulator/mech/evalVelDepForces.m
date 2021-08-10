% Evaluates the velocity dependent and Coriolis term of the mechanical system

% Input
%  th:      Rod angles w.r.t. x axis
%  thd:     Rod angle rates
%  MECH:    Structure with system parameters
%   .L:         Length of first rod (m)
%   .L23:       Length of second rod (m)
%   .m:         First rod mass, distributed (kg)
%   .mh:        Point mass at the end of second rod (kg)
%   .mp:        Point mass at the end of first rod (kg)

% Ouput
%  fc:      Velocity-dependent and Coriolis forces term (2 x 1)

function fc = evalVelDepForces(th, thd, MECH)

% Retrieve angles and angle rates
th1     = th(1);
th2     = th(2);
th1d    = thd(1);
th2d    = thd(2);

% Evaluate
fc = MECH.L * MECH.L23 * MECH.mh * sin(th1-th2) * [th2d^2; - th1d^2];

end