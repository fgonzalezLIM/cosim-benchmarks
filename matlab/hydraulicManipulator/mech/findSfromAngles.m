% Returns the actuator length, its rate, and the Jacobian matrix that relates sd and 
% the angular velocities of the mechanism

% Input
%  th:      Joint angles
%  thd:     Joint rates
%  MECH: 	System properties
%   .L:         Length of first rod
%  s:       Cylinder length
%  sd:      Cylinder length rate
%  Ai:      Jacobian matrix

% Output
%  s:       Actuator length (m)
%  sd:      Actuator rate (m/s)
%  Ai:      Interface Jacobian (1 x 2)

function [s, sd, Ai] = findSfromAngles(th,thd,MECH)

% Retrieve angles and angle rates
th1     = th(1);
th1d    = thd(1);
th2d    = thd(2);

% Evaluate

% Cylinder length
s = MECH.L * sqrt(1.0 - sqrt(3.0)/2 * cos(th1));

% Jacobian matrix
Ai = [sqrt(3.0)*MECH.L^2/4.0 * sin(th1)/s, 0];

% Cylinder length rate
sd = Ai * [th1d;th2d];

end