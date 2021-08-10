function [SYS] = getManipulatorProperties(scenario)

% This function provides the parameters of the 2 d.o.f. hydraulic manipulator.
%
% Input
%  scenario:    Manoeuvre to be simulated
%
% Output
%  SYS:         Structure with system properties, including;


% ______________________________________________________________________________ Manoeuvre
SYS.inputK  = scenario;            % 0: steps; 1: sin
SYS.th      = deg2rad(30.0);       % Initial angle of the first rod w.r.t x axis
SYS.th2 	= deg2rad(-90.0);	   % Initial angle of the second rod w.r.t. x axis
SYS.s0      = 0.5;                 % Initial length of actuator (m)

% ____________________________________________________________________ Mechanical assembly
SYS.L       = 1.0;          % Length of first rod (m)
SYS.L23     = SYS.L/2.0;    % Length of second rod (m)
SYS.m       = 200.0;        % First rod mass, distributed (kg)
SYS.mp      = 250.0;        % Point mass at the end of first rod (kg)
SYS.mh      = 100.0;        % Point mass at the end of second rod (kg)
SYS.g       = 9.81;         % Gravity (m/s^2)

SYS.xA  = 0.0;              % x coordinate of fixed point A
SYS.yA  = 0.0;              % y coordinate of fixed point A
SYS.xB  = sqrt(3.0)/2.0;    % x coordinate of fixed point B
SYS.yB  = 0.0;              % y coordinate of fixed point B

% _____________________________________________________________________________ Hydraulics
SYS.A       = 0.0065;       % Piston area (m^2)
SYS.Lc      = 0.442;        % Piston length (m) when retracted
SYS.c       = 1.0e5;        % Viscous friction coefficient in actuator (Ns/m)
SYS.cd      = 0.67;         % Valve discharge coefficient (-)
SYS.rho     = 850.0;        % Fluid density (kg/m^3)
SYS.pp      = 7.6e6;        % Pump pressure (Pa)
SYS.pt      = 0.1e6;        % Tank pressure (Pa)

% Bulk modulus parameters (Taken from Cardona1990)
SYS.beta_a  = 6.53e-10;     % (Pa) 
SYS.beta_b  = -1.19e-18;    % (-)

end