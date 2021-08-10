% Function that evaluates the partial derivatives of the pressure rates w.r.t. the 
% system coordinates

% Input
%  q:       Generalized coordinates of the system
%  p:       Hydraulic pressures
%  qd:      Generalized velocities of the system
%  kappa:   Spool displacement
%  SYS:     Structure with system information
%   .Lc:        Piston length (m)
%   .s0:        Initial length of actuator (m)

% Output
%  dhdq:    Partial derivatives matrix (2 x 7) 

function dhdq = evaldhdq(q,p,qd,kappa,SYS)

% Preallocate and evaluate variables
dhdq = zeros(2, 7);

l_1     = 0.5*SYS.Lc + SYS.s0 - q(7);
l_2     = 0.5*SYS.Lc + q(7) - SYS.s0;

% Eval pressure rates
[h1,h2] = evalPressureRates(q(7),qd(7),p,kappa,SYS);

% Update
dhdq(1,7) = h1/l_1;
dhdq(2,7) = -h2/l_2;

end