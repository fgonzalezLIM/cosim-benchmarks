% Performs the manager functions for the hydraulic manipulator.

% y1: s, sd - Outputs from mechanical system
% y2: p1, p2, fh - Outputs from hydraulics

function MGR = managerEval(MGR, y1, y2, t)

% ___________________________________________________________________ Store system outputs

s   = y1(1);
sd  = y1(2);
p1  = y2(1);
p2  = y2(2);
fh  = y2(3);

MGR.counter = MGR.counter + 1;
if (MGR.counter == MGR.saveEvery)
    MGR.storeIdx = MGR.storeIdx + 1;
    MGR.STORE.t(MGR.storeIdx, 1)        = t;
    MGR.STORE.s(MGR.storeIdx, 1)        = s;
    MGR.STORE.sd(MGR.storeIdx, 1)       = sd;
    MGR.STORE.p1(MGR.storeIdx, 1)       = p1;
    MGR.STORE.p2(MGR.storeIdx, 1)       = p2;
    MGR.STORE.fh(MGR.storeIdx, 1)       = fh;
    
    % Reset counter
    MGR.counter = 0;
end

% ____________________________________________________________ Update inputs to subsystems

MGR.uEx1 = [p1; p2; fh];
MGR.uEx2 = [s; sd];

end