% This function co-simulates a set of 2 subsystems in an explicit, single-rate Jacobi
% scheme.

% Input
%  MGR:         Manager structure, defined in managerCreate.m
%  ss1:         Subsystem 1
%  ss2:         Subsystem 2
%  OPT:         Co-simulation options, including:
%   .finalT     Final simulation time
%
% Output
%  MGR:         Updated manager structure, includes stored results
%  ss1:         Updated subsystem 1
%  ss2:         Updated subsystem 2

function [MGR, ss1, ss2] = simulate_jacobi_SR(MGR, ss1, ss2, OPT)

t                       = 0.0;
t1                      = 0.0;
t2                      = 0.0;
stepsTaken1             = 0;
stepsTaken2             = 0;
MGR.repI                = MGR.reportEvery - 1;

while t < OPT.finalT
    
    % Report when required
    MGR.repI = MGR.repI + 1;
    if (MGR.repI == MGR.reportEvery)
        Xt = ['Macro time: ', num2str(t), '.']; disp(Xt);
        MGR.repI = 0;
    end
    
    % Exchange coupling variables and call manager
    y1 = ss1.readOutputs;
    y2 = ss2.readOutputs;
    
    % This function is case-dependent, and should be defined by the user
    MGR = managerEval(MGR, y1, y2, t);
    
    ss1.setInputs(MGR.uEx1);
    ss2.setInputs(MGR.uEx2);
    
    % Increase goal time
    t = t + MGR.H;
    
    % Advance integrations
    while (t1 < t)
        t1 = t1 + MGR.H1;
        doStep(ss1, t1);
        stepsTaken1 = stepsTaken1 + 1;
    end
    while (t2 < t)
        t2 = t2 + MGR.H2;
        doStep(ss2, t2);
        stepsTaken2 = stepsTaken2 + 1;
    end
    
    if (stepsTaken1 ~= stepsTaken2); error("Synchro lost");end
    
end

end