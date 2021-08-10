function kappa = updateKappa(t, SYS)

% Updates the spool displacement during motion.
% This motion is kinematically guided, and given as a function of time.

% Input
%  t:           Time
%  SYS:         Structure with system information
%   .inputK     Manoeuvre: 0-steps, 1-sinusoidal
%   .kappa0     Initial value of the spool displacement

% Output
%  kappa:       Spool displacement


% Steps
if (SYS.inputK == 0)

    ramptime = 0.001;

    if (t <= 2.0)
        kappa = SYS.kappa0;
    elseif (t <= 2.0 + ramptime) 
        kappa = SYS.kappa0 - 0.01*(t-2.0)/ramptime;
    elseif (t <= 6.0)
        kappa = SYS.kappa0 - 0.01;
    elseif (t <= 6.0 + 2*ramptime)
        kappa = SYS.kappa0 - 0.01 + 0.03*(t-6.0)/(2*ramptime);
    else
        kappa = SYS.kappa0 + 0.02;
    end

% Sinus
elseif (SYS.inputK == 1)
    
    omega = 2.0;
    ampC = 0.1;
    
    % Increase amplitude gradually beginning at zero until ampC
    if (t <= 1.0)
        amplitude = ampC*t;
    elseif (t <= 8.0)
        amplitude = ampC;
    elseif (t <= 9.0)
        amplitude = ampC*(9.0-t);
    else
        amplitude = 0.0;
    end
    
    kappa = SYS.kappa0 * (1.0 - amplitude*(sin(2.0*pi*omega*t))); 
    
else
    error('Unknown input profile');
end