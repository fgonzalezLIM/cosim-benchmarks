% This script simulates the motion of a 2-d.o.f. linear oscillator.
%
% A co-simulation approach is used.

%                |-> x1             |-> x2
%   |         |----|             |----|         |
%   |--/\/\/--| m1 |----/\/\/----| m2 |--/\/\/--|
%   | k1, c1  |----|   k_c, c_c  |----|  k2, c2 |

% The system is divided into two subsystems: SS1 and SS2.

clear variables
close all

% __________________________________________________ Extend path to find library functions
addpath("../common");

% ________________________________________________________________________ Define scenario

scenario        = 0;        % Simulation case to be solved - available: 0, 1.
compareResults  = 1;        % If 1, compare results to analytic and monolithic solutions

% __________________________________________________________________ Simulation parameters

finalT      = 10.0;         % Final time of motion (s)
reportEvery = 5.0e-1;       % Display time in screen to show progress every # steps
saveEvery   = 1;            % Store results every # manager steps

% Macro step-sizes
H           = 1.0e-3;       % Manager   
H1          = 1.0e-3;       % SS1
H2          = 1.0e-3;       % SS2
% Micro (internal) step-sizes
dt1         = 1.0e-3;       % SS1     
dt2         = 1.0e-3;       % SS2

% Co-simulation options
COSIMOPTS.finalT    = finalT;
COSIMOPTS.scheme    = 'JacobiSR';   % JacobiSR, JacobiMR

% _____________________________________________________________ Create co-simulation units

% Options for the definition of the subsystems
OPT1.scenario   = scenario;
OPT1.integrator = 1;            % 1: FWE, 2: TR
OPT1.H          = dt1;
% Only if TR is used
OPT1.maxIter    = 10;
OPT1.maxError   = 1.0e-9;

OPT2.scenario   = scenario;
OPT2.integrator = 1;            % 1: FWE, 2: TR
OPT2.H          = dt2;
% Only if TR is used
OPT2.maxIter    = 10;
OPT2.maxError   = 1.0e-9;

% The input/output of these may be either force ('F') or positions and velocities ('S')
ss1 = mass_ss('SS1', 1, 'S', 'F', OPT1);     % Number, input, output, options
ss2 = mass_ss('SS2', 2, 'F', 'S', OPT2);     % Number, input, output, options

% Manager creation
MANOPT.H1           = H1;
MANOPT.H2           = H2;
MANOPT.saveEvery    = saveEvery;
MANOPT.reportEvery  = round(reportEvery/H);
MGR                 = managerCreate(H, MANOPT);

% _________________________________________________________________________ Initialization

% Retrieve outputs of subsystems
y1 = ss1.readOutputs;
y2 = ss2.readOutputs;

MGR = managerEval(MGR, y1, y2, 0.0);

% Send inputs to subsystems
ss1.setInputs(MGR.uEx1);
ss2.setInputs(MGR.uEx2);

% Initialize units
ss1.initialize(0.0);
ss2.initialize(0.0);


% ______________________________________________________________________________ Execution

% Simulate

if (strcmp(COSIMOPTS.scheme,'JacobiSR') == true )
    
    assert(H==H1, "Single-rate execution requires matching step-sizes.")
    assert(H==H2, "Single-rate execution requires matching step-sizes.")
    
    tic % Explicit Jacobi, single rate
    [MGR, ss1, ss2] = simulate_jacobi_SR(MGR, ss1, ss2, COSIMOPTS);
    toc
elseif (strcmp(COSIMOPTS.scheme,'JacobiMR') == true )
    tic % Explicit Jacobi, multi-rate
    [MGR, ss1, ss2] = simulate_jacobi_MR(MGR, ss1, ss2, COSIMOPTS);
    toc   
else
    error('Unrecognized co-simulation scheme');
end

% ________________________________________________________________________ Post-processing

if (compareResults == 1)
    
    % Compare the co-simulation results to those delivered by the analytic and monolithic
    % solution methods.

    SYS         = getOscillatorProperties(scenario);

    % Monolithic integration: specify these options
    % Options structure for the integration
    OPTIONS.saveEvery   =   1;     % Storage rate
    OPTIONS.integrator  =   1;     % 1: Forward-Euler; 2: Trapezoidal rule

    % If using TR as integrator, add these parameters
    if OPTIONS.integrator == 2
        OPTIONS.maxIter     = 10;       % Maximum number of iterations
        OPTIONS.maxError    = 1.0e-7;   % Maximum x increase between correction iterations
    end

    RES_mono        = simulate_monolithic(H, finalT, OPTIONS, SYS);
    RES_analytic    = simulate_analytic(H, finalT, SYS);
    
    % Evaluate mechanical energy
    RES_mono        = evalMechEnergy(RES_mono, SYS);
    RES_analytic    = evalMechEnergy(RES_analytic, SYS);
    MGR.STORE       = evalMechEnergy(MGR.STORE, SYS);

    indexPlots = 0;
    indexPlots = plotResults(MGR.STORE, RES_analytic, RES_mono, indexPlots);

end

% ___________________________________________________________________ Restore initial path
rmpath("../common");