function dhdp = evaldhdp(s, p, kappa, SYS)

% Function that evaluates the partial derivative of the pressure rates with
% respect to the pressures in the cylinder.

% Input
%  s:       Cylinder displacement
%  p:       Hydraulic pressures in the cylinder
%  kappa:   Spool displacement
%  SYS:     Structure with system information
%   .A:         Cylinder area
%   .cd:        Valve discharge coefficient
%   .Lc:        Maximum piston length
%   .pp:        Pump pressure
%   .pt:        Tank pressure
%   .rho:       Fluid density
%   .s0:        Initial length of actuator

% Output
%  dhdp:    Partial derivatives matrix (2 x 2)

% ___________________________________________________________ Retrieve vars
A       = SYS.A;
Lc      = SYS.Lc;
cd      = SYS.cd;
pp      = SYS.pp;
pt      = SYS.pt;
s0      = SYS.s0;
rho     = SYS.rho;

p1      = p(1); 
p2      = p(2); 

% __________________________________________________ Intermediate variables

% Evaluate lengths
l_1     = 0.5*Lc + s0 - s;
l_2     = 0.5*Lc + s - s0;

% Evaluate areas
Ai      = 0.0005*kappa;
Ao      = 0.0005*(1.0-kappa);

% Pressure values
VALp1   = (pp-p1);
VALp2   = (pp-p2);
VAL1T   = (p1-pt);
VAL2T   = (p2-pt);

deltap1 = 1.0;
deltap2 = 1.0;
deltat1 = 1.0;
deltat2 = 1.0;

if (VALp1 < 0); deltap1 = 0; end
if (VALp2 < 0); deltap2 = 0; end
if (VAL1T < 0); deltat1 = 0; end
if (VAL2T < 0); deltat2 = 0; end

D = 1.0/l_1 * (Ai/sqrt(pp-p1)*deltap1 + Ao/sqrt(p1-pt)*deltat1);
E = 1.0/l_2 * (Ao/sqrt(pp-p2)*deltap2 + Ai/sqrt(p2-pt)*deltat2);

beta1 = evalbeta(p1,SYS);
beta2 = evalbeta(p2,SYS);

% ________________________________________________________________ Evaluate

dhdp = -(cd)/(A*sqrt(2.0*rho)) * [beta1*D 0; 0 beta2*E];
