% This function provides a simulation of the motion of a 2 d.o.f. linear oscillator.
%
% A monolithic integration approach is used.
%
% Input
%  H:           Integration step-size (s)
%  finalT:      Final time of motion (s)
%  OPTIONS:     Structure with simulation options, including:
%   .saveEvery  Store results every # steps
%   .integrator Integration formula: 1 = forward Euler; 2 = TR
%  SYS:         Structure with system properties (see getOscillatorProperties.m)
%
% Output
%  RES:         Structure with results, including:
%   .t          Timestamps (s)
%   .x          Positions (m)
%   .xd         Velocities (m/s)


%                |-> x1             |-> x2
%   |         |----|             |----|         |
%   |--/\/\/--| m1 |----/\/\/----| m2 |--/\/\/--|
%   | k1, c1  |----|   k_c, c_c  |----|  k2, c2 |

function RES = simulate_monolithic(H, finalT, OPTIONS, SYS)

% Pass integration step-size as OPTIONS parameter
OPTIONS.H = H;

% ___________________________________________________ System parameters and initial state

% Initial configuration
x   = SYS.x0;
xd  = [SYS.xd0(1);SYS.xd0(2)]; 

% __________________________________________________________________ Pre-allocate storage
npoints         = round(max(finalT/(H*OPTIONS.saveEvery), 1)); 
RES.t           = zeros(npoints, 1);
RES.x           = zeros(npoints, 2);
RES.xd          = zeros(npoints, 2);
RES.xdd         = zeros(npoints, 2);

% _____________________________________________________________________________ Integrate
t           = 0.0;
i           = OPTIONS.saveEvery - 1;
storeIdx    = 0;

while t <= finalT
    
    % Evaluate acceleration
    xdd = evalAcceleration(SYS, x, xd);
    
    % Store results
    i = i + 1;
	if (i == OPTIONS.saveEvery)
        storeIdx = storeIdx + 1;
        RES.t(storeIdx)         = t;
        RES.x(storeIdx,:)       = x;
        RES.xd(storeIdx,:)      = xd;
        RES.xdd(storeIdx,:)     = xdd;
        i = 0;
	end
    
    % Integrate
    if OPTIONS.integrator == 1
        [x, xd] = integrate_FWE(x, xd, xdd, OPTIONS);
    elseif OPTIONS.integrator == 2
        [x, xd] = integrate_TR(x, xd, xdd, OPTIONS, SYS, @evalAcceleration);
    else
        error('Unrecognized integration formula');
    end
        
    % Increase time
    t = t + H;
end

end

% __________________________________________________________ Auxiliary: eval acceleration
function xdd = evalAcceleration(SYS, x, xd)
    % Coupling force
    fc_sp = [1;-1] * SYS.kc*(x(2)-x(1));
    fc_dp = [1;-1] * SYS.cc*(xd(2)-xd(1));
    
    % Evaluate the rest of the forces
    f_sp = -SYS.K*x;
    f_dp = -SYS.C*xd;
    
    % Evaluate accelerations
    sol     = SYS.M\(f_sp + f_dp + fc_sp + fc_dp);
    xdd     = sol(1:2);
end
