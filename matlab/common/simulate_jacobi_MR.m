% This function co-simulates a set of 2 subsystems in an explicit, multi-rate Jacobi
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

% ### The file is under development. Calls to the manager inside the execution loops
% should be implemented to enable extrapolation orders higher than ZOH.

function [MGR, ss1, ss2] = simulate_jacobi_MR(MGR, ss1, ss2, OPT)

t                       = 0.0;
t1                      = 0.0;
t2                      = 0.0;
MGR.repI                = MGR.reportEvery - 1;

while t < OPT.finalT
    
    % Report when required
    MGR.repI = MGR.repI + 1;
    if (MGR.repI == MGR.reportEvery)
        Xt = ['Macro time: ', num2str(t), '.']; disp(Xt);
        MGR.repI = 0;
    end
    

   % Xt = ['Macro time: ', num2str(t), '.']; disp(Xt);

    % Exchange coupling variables and call manager
    % In matching grids, these are always available at communication points
    % The "if" conditions ensure parallel execution regardless of the relative size of
    % macro and micro steps. Outputs are only read 
    if ((MGR.H < MGR.H1) && (MGR.H >= MGR.H2) )
        if (t1 <= t + MGR.H1/2); y1 = ss1.readOutputs; end
        y2 = ss2.readOutputs;
    elseif ((MGR.H > MGR.H2) && (MGR.H <= MGR.H1) )
        y1 = ss1.readOutputs;
        y2 = ss2.readOutputs;
    else
        error("Combination of macro/micro steps is not admissible")
    end
    MGR = managerEval(MGR, y1, y2, t);

    % Increase goal time
    t = t + MGR.H;

    %Xt = ['Goal time: ', num2str(t), '.']; disp(Xt);

    % Advance integrations
    while (t1 < t)
        %Xt = ['Eval SS1 at time: ', num2str(t1), '.']; disp(Xt);
        MGR = managerEval(MGR, y1, y2, t1);
        ss1.setInputs(MGR.uEx1);
        t1 = t1 + MGR.H1;
        doStep(ss1, t1);
    end
    while (t2 < t)
        %Xt = ['Eval SS2 at time: ', num2str(t2), '.']; disp(Xt);
        MGR = managerEval(MGR, y1, y2, t2);
        ss2.setInputs(MGR.uEx2);
        t2 = t2 + MGR.H2;
        doStep(ss2, t2);
    end
     
end

end