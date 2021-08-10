% This script simulates the motion of a 2-d.o.f. hydraulic manipulator.
%
% A co-simulation approach is used.
%
% The system is divided into two subsystems: hydraulic_ss (for the hydraulics subsystem) 
% and manipulator_ss (for the multibody system dynamics of the mechanical system).

clear variables
close all

% __________________________________________________ Extend path to find library functions
addpath("../common");
addpath("./mech");
addpath("./hyd");

% ________________________________________________________________________ Define scenario

scenario        = 0;        % Simulation case to be solved: 0-steps, 1-sinusoidal.
compareResults  = 1;        % If 1, compare results to analytic and monolithic solutions

% __________________________________________________________________ Simulation parameters

finalT      = 10.0;         % Final time of motion (s)
reportEvery = 5.0e-1;       % Display time in screen to show progress every # steps
saveEvery   = 1;            % Store results every # manager steps

% Macro step-sizes
H           = 1.0e-3;      % Manager   
H1          = 1.0e-3;      % MBS
H2          = 2.0e-4;      % Hyd
% Micro (internal) step-sizes
dt1         = 1.0e-3;      % MBS     
dt2         = 2.0e-4;      % Hyd

% Co-simulation options
COSIMOPTS.finalT    = finalT;
COSIMOPTS.scheme    = 'JacobiMR';   % JacobiSR, JacobiMR

% _____________________________________________________________ Create co-simulation units

% Subsystem 1: MBS model
OPT1.integrator = 1;            % 1: FWE, 2: TR
OPT1.H          = dt1;
% Only if TR is used
OPT1.maxIter    = 25;
OPT1.maxError   = 1.0e-7;

% Subsystem 2: Hydraulics
OPT2.scenario   = scenario;
OPT2.integrator = 1;            % 1: FWE, 2: TR
OPT2.H          = dt2;
% Only if TR is used
OPT2.maxIter    = 10;
OPT2.maxError   = 1.0e-9;

% Create subsystems
mech_ss1    = manipulator_ss('mech', 'F', OPT1);
hydr_ss2    = hydraulic_ss('hyd', 'F', OPT2);

% Manager creation
MANOPT.H1           = H1;
MANOPT.H2           = H2;
MANOPT.saveEvery    = saveEvery;
MANOPT.reportEvery  = round(reportEvery/H);
MGR                 = managerCreate(H, MANOPT);

% _________________________________________________________________________ Initialization

% The initialization of the hydraulics requires to know the following
% values that must be provided by the mechanical sub-system
%   - Actuator length s
%   - Actuator length rate sd
%   - Effective force (to evaluate the initial static equilibrium)
% For these reasons, the mechanical sub-system is evaluated first and its
% outputs sent as inputs to the hydraulics sub-system

% Retrieve outputs from the mechanics and send to hydraulics
yMech = mech_ss1.readOutputs();
hydr_ss2.setInputs(yMech);

% Evaluate effective force of mechanics (for initialization only)
feff0 = mech_ss1.evalEffectiveForce;
hydr_ss2.setEffectiveForce(feff0);

% Initialize hydraulics
hydr_ss2.initialize(0.0);

% Retrieve hydraulics outputs and send to mechanics
yHyd = hydr_ss2.readOutputs();
mech_ss1.setInputs(yHyd);

% Initialize mechanics
mech_ss1.initialize(0.0);

% Initialize manager
yMech = mech_ss1.readOutputs;
yHyd = hydr_ss2.readOutputs;

% ______________________________________________________________________________ Execution

% Simulate
if (strcmp(COSIMOPTS.scheme,'JacobiSR') == true )
    
    assert(H==H1, "Single-rate execution requires matching step-sizes.")
    assert(H==H2, "Single-rate execution requires matching step-sizes.")
    
    tic % Explicit Jacobi, single rate
    [MGR, mech_ss1, hydr_ss2] = simulate_jacobi_SR(MGR, mech_ss1, hydr_ss2, COSIMOPTS);
    toc
elseif (strcmp(COSIMOPTS.scheme,'JacobiMR') == true )
    tic % Explicit Jacobi, multi-rate
    [MGR, mech_ss1, hydr_ss2] = simulate_jacobi_MR(MGR, mech_ss1, hydr_ss2, COSIMOPTS);
    toc   
else
    error('Unrecognized co-simulation scheme');
end

mech_ss1.terminate();
hydr_ss2.terminate();

% ________________________________________________________________________ Post-processing

if (compareResults == 1)
    
    % Compare the co-simulation results to those delivered by the analytic and monolithic
    % solution methods.

    SYS         = getManipulatorProperties(scenario);
    
	% Monolithic integration: specify these options
    % Options structure for the integration
    OPTIONS.saveEvery   =   1;     % Storage rate
    OPTIONS.integrator  =   1;     % 1: Forward-Euler; 2: Trapezoidal rule

    % Integration parameters
   	OPTIONS.maxIter     = 100;      % Maximum number of Newton-Raphson iterations
    OPTIONS.maxError    = 1.0e-5;   % Maximum error in Newton-Raphson iteration
    OPTIONS.nProjs      = 3;        % Number of velocity and accel. projections
    OPTIONS.alpha       = 1.0e12;   % Penalty factor for augmented Lagrangian method

    addpath("./mono");
    RES_mono        = simulate_monolithic(H, finalT, OPTIONS, SYS);
    rmpath("./mono");
    
    indexPlots = 0;
    indexPlots = plotResults(MGR.STORE, RES_mono, indexPlots);
    
end


% ___________________________________________________________________ Restore initial path
rmpath("./hyd");
rmpath("./mech");
rmpath("../common");