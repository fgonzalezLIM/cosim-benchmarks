function MGR = managerEval(MGR, y1, y2, t)

% Performs the manager functions for the linear oscillator.
% Three operation modes are defined:
%   1. Initialization (t = 0)
%   2. Macro step (communication point)
%   3. Micro step (intermediate points)

% y1: s, sd, force
% y2: s, sd, force

% ________________________________________________________________________________________
%                                                                    Select operation mode

% Operation mode 1: initialization, 2: macro-step (communication point), 3: micro-step
if (t < MGR.Hmin/2)
    opMode = 1;  % At t = 0.0 the manager is initialized
else
    lastT = MGR.STORE.t(end);
    if (t >= lastT + MGR.H)
        opMode = 2;
    else
        opMode = 3;
    end
end

% ________________________________________________________________________________________
%                                                                     Store system outputs

% Only at macro steps and initialization
if ((opMode == 1) || (opMode == 2))

    q   = [y1(1); y2(1)];
    qd  = [y1(2); y2(2)];
    f   = [y1(3); y2(3)];

    MGR.counter = MGR.counter + 1;
    if (MGR.counter == MGR.saveEvery)
        if (opMode == 2); MGR.storeIdx = MGR.storeIdx + 1; end
        MGR.STORE.t(MGR.storeIdx, 1)        = t;
        MGR.STORE.x(MGR.storeIdx, :)        = q;
        MGR.STORE.xd(MGR.storeIdx, :)       = qd;
        MGR.STORE.fc(MGR.storeIdx, :)       = f;

        % Reset counter
        MGR.counter = 0;
    end
    
    % Create extrapolation storage during initialization
    % Update it during operation
    if (opMode == 1)      
        for i=1:MGR.order1+1; MGR.extrBuf1(:,i) = [0.0;y1]; end
        for i=1:MGR.order2+1; MGR.extrBuf2(:,i) = [0.0;y2]; end
    else
        % Move values to previous columns
        % and update
        for i=1:MGR.order1; MGR.extrBuf1(:,i) = MGR.extrBuf1(:,i+1); end
        for i=1:MGR.order2; MGR.extrBuf2(:,i) = MGR.extrBuf2(:,i+1); end
        MGR.extrBuf1(:,MGR.order1+1) = [t;y1];
        MGR.extrBuf2(:,MGR.order2+1) = [t;y2];
    end
    
end

% ________________________________________________________________________________________
%                                                              Update inputs to subsystems

% Retrieve extrapolated values
[y1ex, y2ex] = extrapolate(MGR,t);

% Note that the force sign is changed
MGR.uEx1 = [y2ex(1:2);-y2ex(3)];
MGR.uEx2 = [y1ex(1:2);-y1ex(3)];

end

% ________________________________________________________________________________________
%                                                                   Extrapolation function
function [y1ex, y2ex] = extrapolate(MGR,t)
   
    % Determine extrapolation method - we need enough stored points to approximate
    order1 = min(MGR.storeIdx-1, MGR.order1);
    order2 = min(MGR.storeIdx-1, MGR.order2);
    
    y1ex = extrapolationDetail(MGR.extrBuf1, t, order1);
    y2ex = extrapolationDetail(MGR.extrBuf2, t, order2);

end

function [yex] = extrapolationDetail(xV, t, order)

    x = xV(1,:);
    y = xV(2:end, :);
    yex = zeros(size(y,1),1);

    if order == 0
        yex = y(:, end);
    elseif order == 1
        yex = (interp1(x(:,end-1:end)', y(:,end-1:end)', t, 'linear', 'extrap'))';
    elseif order == 2
        for i=2:size(xV,1)
            p  = polyfit(x',xV(i,:),2);
            yex(i-1,:)   = polyval(p,t);
        end
    end
end
