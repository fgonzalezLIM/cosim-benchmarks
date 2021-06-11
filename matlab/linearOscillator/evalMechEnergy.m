function [STORAGE] = evalMechEnergy(STORAGE, SYS)

% Evaluates the mechanical energy of a 2 d.o.f. linear oscillator.

% Input
%  STORAGE:     Storage structure. It must include these fields (of the same length):
%   .t              Array of timesteps
%   .x              Array of positions
%   .xd             Array of velocities
%  SYS:         Structure with the mechanical properties of the linear oscillator
%               (created in getOscillatorProperties.m)
%
% Output
%  STORAGE:     These new fields are added to it:
%   .kinEn          Kinetic energy
%   .potEn          Potential energy (springs)
%   .mechEn         Mechanical energy (kinEn + potEn)

% Pre-allocate arrays
STORAGE.kinEn   = 0*STORAGE.t;
STORAGE.potEn   = 0*STORAGE.t;

% Mass and stiffness matrices
M = [SYS.m(1) 0.0; 0.0 SYS.m(2)];
K = [SYS.k(1)+SYS.kc, -SYS.kc; -SYS.kc, SYS.k(2)+SYS.kc];

% For each timestep, evaluate kinetic and potential energy
for i=1:length(STORAGE.t)
    STORAGE.kinEn(i) = 0.5 * STORAGE.xd(i,:) * M * STORAGE.xd(i,:)';
    STORAGE.potEn(i) = 0.5 * STORAGE.x(i,:) * K * STORAGE.x(i,:)';
end

STORAGE.mechEn = STORAGE.kinEn + STORAGE.potEn;

end