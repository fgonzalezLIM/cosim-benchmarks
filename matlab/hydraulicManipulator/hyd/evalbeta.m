function beta = evalbeta(p, HYD)

% Evaluates the bulk modulus of the fluid as a function of beta.
% This evaluation follows A. Cardona and M. Geradin, Modeling of a hydraulic actuator in
% flexible machine dynamics simulation, Mechanism and Machine Theory, 25(2):193-207, 1990.

% Input
%  p:       fluid pressure
%  SYS:     structure with system information
%   .a:         Bulk modulus parameter
%   .b:         Bulk modulus parameter

% Output
%  beta:    Bulk modulus

a       = HYD.beta_a;
b       = HYD.beta_b;
beta    = (1.0 + a*p + b*p^2) / (a + 2.0*b*p);

