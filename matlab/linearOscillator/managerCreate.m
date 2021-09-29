function MGR = managerCreate(H, OPTS)

% Creates the manager for the linear oscillator.
%
% The manager is responsible for handling input/output management, e.g., extrapolation,
% corrections, etc.
%
% It can also be used to store information.
%
%
% Parameters -
% H:                Macro step-size
% OPTS:             Options
%   .H1:            Macro-step of ss1
%   .H2:            Macro-step of ss2
%   .order1:        Extrapolation order for ss1 (0,1,2)
%   .order2:        Extrapolation order for ss2 (0,1,2)
%   .reportEvery:   Report on screen every # steps
%   .saveEvery:     Store data every # steps

% ________________________________________________________________________________________
%                                                                             Verification

assert (H > 0.0, "Macro step-size must be greater than zero");
assert (OPTS.H1 > 0.0, "Micro step-size 1 must be greater than zero");
assert (OPTS.H2 > 0.0, "Micro step-size 2 must be greater than zero");

assert ((OPTS.order1 >=0) && (OPTS.order1) <=2, "Extrapolation order should be 0, 1, or 2");
assert ((OPTS.order2 >=0) && (OPTS.order2) <=2, "Extrapolation order should be 0, 1, or 2");

% ________________________________________________________________________________________
%                                                                           Initialization

% Create structures for storage
MGR.STORE       = [];       
MGR.storeIdx    = 1;
MGR.extrBuf1    = [];
MGR.extrBuf2    = [];

% Options
MGR.H           = H;
MGR.H1          = OPTS.H1;
MGR.H2          = OPTS.H2;
MGR.Hmin        = min([MGR.H;MGR.H1;MGR.H2]); % Minimum step-size involved 
MGR.saveEvery   = OPTS.saveEvery;
MGR.reportEvery = OPTS.reportEvery;
MGR.counter     = MGR.saveEvery - 1; % Always store first value

% Extrapolation orders: indexes are swapped, because input for subsystem 1 is output from
% 2 and vice versa
MGR.order1      = OPTS.order2;          
MGR.order2      = OPTS.order1;


end