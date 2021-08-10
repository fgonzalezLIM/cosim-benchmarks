function MGR = managerCreate(H, OPTS)

% Creates the manager for the hydraulic manipulator.
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
%   .reportEvery:   Report on screen every # steps
%   .saveEvery:     Store data every # steps

% Create structures for storage
MGR.STORE       = [];       
MGR.storeIdx    = 0;
MGR.H           = H;
MGR.H1          = OPTS.H1;
MGR.H2          = OPTS.H2;
MGR.saveEvery   = OPTS.saveEvery;
MGR.reportEvery = OPTS.reportEvery;
MGR.counter     = MGR.saveEvery - 1; % Always store first value

end