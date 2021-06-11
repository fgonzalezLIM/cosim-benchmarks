function MGR = managerEval(MGR, y1, y2, t)

% Performs the manager functions for the linear oscillator.

% y1: s, sd, force
% y2: s, sd, force

% ________________________________________________________________________________________
%                                                                     Store system outputs

q   = [y1(1); y2(1)];
qd  = [y1(2); y2(2)];
f   = [y1(3); y2(3)];

MGR.counter = MGR.counter + 1;
if (MGR.counter == MGR.saveEvery)
    MGR.storeIdx = MGR.storeIdx + 1;
    MGR.STORE.t(MGR.storeIdx, 1)        = t;
    MGR.STORE.x(MGR.storeIdx, :)        = q;
    MGR.STORE.xd(MGR.storeIdx, :)       = qd;
    MGR.STORE.fc(MGR.storeIdx, :)       = f;
    
    % Reset counter
    MGR.counter = 0;
end

% ________________________________________________________________________________________
%                                                              Update inputs to subsystems

% Note that the force sign is changed
MGR.uEx1 = [y2(1:2);-y2(3)];
MGR.uEx2 = [y1(1:2);-y1(3)];

end