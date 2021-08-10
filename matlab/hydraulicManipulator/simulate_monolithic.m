% This function provides a simulation of the motion of the 2 d.o.f. hydraulic manipulator.
%
% A monolithic integration approach is used.
%
% Input
%  H:           Integration step-size (s)
%  finalT:      Final time of motion (s)
%  OPTIONS:     Structure with simulation options, including:
%   .saveEvery  Store results every # steps
%  SYS:         Structure with system properties (see getManipulatorProperties.m)
%
% Output
%  RES:         Structure with results, including:
%   .t          Timestamps (s)
%   .kappa      Spool displacement
%   .pos        Positions (m)
%   .vel        Velocities (m/s)
%   .acc        Accelerations (m/s^2)
%   .F          Actuator force (N)
%   .lambda     Lagrange multipliers
%   .p          Cylinder pressures (Pa)
%   .pd         Time-derivatives of the cylinder pressures (Pa/s)
%   .violC      Violation of kinematic constraints


function RES = simulate_monolithic(H, finalT, OPTIONS, SYS)

% __________________________________________________________________ Simulation properties
saveRate    = OPTIONS.saveEvery;

% Method parameters
maxIter = OPTIONS.maxIter;      % Maximum number of Newton-Raphson iteration
maxError= OPTIONS.maxError;     % Maximum error in Newton-Raphson iteration
nProjs  = OPTIONS.nProjs;       % Number of velocity and accel. projections
alpha   = OPTIONS.alpha;        % Penalty factor for augmented Lagrangian method

% ________________________________________________________________________________ Storage

% Number of storage points
npoints     = round(max(finalT/(H*saveRate), 1));

% Store time
nvars           = 7;
RES.t           = zeros(1, npoints);
RES.kappa       = zeros(1, npoints);
RES.pos         = zeros(nvars, npoints);
RES.vel         = zeros(nvars, npoints);
RES.acc         = zeros(nvars, npoints);
RES.p           = zeros(2, npoints);
RES.pd          = zeros(2, npoints);
RES.lambda      = zeros(nvars - 2, npoints);
RES.F           = zeros(1, npoints);
RES.violC       = zeros(1, npoints);

% __________________________________________________________________ Initial configuration
L  = SYS.L;
th = SYS.th;
s0 = SYS.s0;
q  = [  L/2.0*cos(th); L/2.0*sin(th); ...
        L*cos(th); L*sin(th); ...
        L*cos(th); L*sin(th) - SYS.L23; s0];

% Initial velocities are set to zero
qd  = zeros(nvars,1);

% Evaluate initial pressures
p       = [3.3e6;4.4e6];        % Initial guess
kappa   = 0.5;                  % Spool displacement (valve)
[p, kappa, res] = evalIniPressureMono(p, kappa, q, SYS, 4);

% Compute Initial Force
F = (p(2) - p(1))*SYS.A;

% Store initial value of the spool displacement
SYS.kappa0 = kappa;


% % _____________________________________________________________ Dynamic terms (constant)

% Mass matrix
MM  = [ 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0;
        0, 0, SYS.mp+SYS.m/3, 0, 0, 0, 0;
        0, 0, 0, SYS.mp+SYS.m/3, 0, 0, 0;
        0, 0, 0, 0, SYS.mh, 0, 0;
        0, 0, 0, 0, 0, SYS.mh, 0;
        0, 0, 0, 0, 0, 0, 0];
    
% Damping matrix
C               = zeros(nvars,nvars);
C(nvars,nvars)  = SYS.c;

% Partial derivative of forces w.r.t. pressures
dQdp        = zeros(nvars,2);
dQdp(7,1)   = -SYS.A;
dQdp(7,2)   = SYS.A;

% _______________________________________________________________ Dynamic terms (variable)

% Kinematic constraints
Phi     = evalConstr(q, SYS);
Jac     = evalJacobian(q, SYS);
Jacdq   = evalJacobiandq(q, qd, SYS);
 
% Partial derivative terms
dhdq    = evaldhdq(q,p,qd,kappa,SYS);
dhdqd   = evaldhdqd(q,p,SYS);

% __________________________________________________________________ Initial accelerations

% Evaluate pressure rates (should be zero)
[h1,h2] = evalPressureRates(q(7),qd(7),p,kappa,SYS);
pd = [h1;h2];

% Applied forces
Q   = evalForces(p, qd, SYS);

% Evaluate accelerations (use an augmented Lagrange method)
LEAD_iniacc = [MM Jac'; Jac, zeros(5,5)];
RHS_iniacc  = [Q; -Jacdq];
sol_iniacc  = LEAD_iniacc\RHS_iniacc;
qdd         = sol_iniacc(1:7);
lambda      = sol_iniacc(8:12);

% _____________________________________________________________________ Dynamic simulation

% Initialize
t           = 0.0;              % Time
i           = saveRate-1;       % Storage counters
storeIdx    = 0;

while (t <= finalT)
    
  	% Save values
    i = i+1;
    if (i == saveRate)
        
        t
        storeIdx = storeIdx + 1;
        
        RES.t(storeIdx)         = t;
        RES.kappa(storeIdx)     = kappa;
        RES.pos(:,storeIdx)     = q;  
        RES.vel(:,storeIdx)     = qd; 
        RES.acc(:,storeIdx)     = qdd;
        RES.lambda(:,storeIdx)  = lambda;
        RES.F(storeIdx)         = F;
        RES.violC(storeIdx)     = norm(Phi,2);
        RES.p(:,storeIdx)       = p;
        RES.pd(:,storeIdx)      = pd;
        
        i = 0;
    end
    
    % Update the valve parameter (use the time for the next step, because the equilibrium
    % is computed at t+H).
    kappa       = updateKappa(t+H, SYS);

    % Predictor
    qdOldBow    = -( 2.0/H * q +  qd );
    qddOldBow   = -( 4.0/H^2 * q  + 4.0/H * qd + qdd);
    pdOldBow    = -( 2.0/H * p +  pd ); 

    q_n1        = q + H * qd + (H^2/2.0) * qdd;
    qd_n1       = qd + H * qdd;
    qdd_n1      = qdd;
    p_n1        = p + H * pd;
    pd_n1       = pd;
    lambda_n1   = lambda;

    % Corrector
    n_iter = 0;
    while n_iter <= maxIter

        % Re-evaluate dynamic terms
        Phi     = evalConstr(q_n1, SYS);
        Jac     = evalJacobian(q_n1, SYS);
        Jacdq   = evalJacobiandq(q_n1, qd_n1, SYS);
        Q       = evalForces(p_n1, qd_n1, SYS);
        [h1,h2] = evalPressureRates(q_n1(7),qd_n1(7),p_n1,kappa,SYS);
        h       = [h1;h2];
        dhdq    = evaldhdq(q_n1,p_n1,qd_n1,kappa,SYS);
        dhdqd   = evaldhdqd(q_n1,p_n1,SYS);
        dhdp    = evaldhdp(q_n1(7), p_n1, kappa, SYS);
        
        % Update Lagrange multipliers
        lambda_n1 = lambda_n1 + alpha*Phi; 

        % Eval residual
        f = H^2*0.25*...
            [MM*qdd_n1 + Jac'*alpha*Phi + Jac'*lambda_n1 - Q;
             pd_n1 - h];

        % Eval tangent matrix
        dfdq11 = MM + H/2.0*C + H^2/4.0*(Jac'*alpha*Jac);
        dfdq12 = -H^2/4.0 * dQdp;
        dfdq21 = -H/2.0*(H/2.0*dhdq + dhdqd);
        dfdq22 = H/2.0 * (eye(2) - H/2.0*dhdp);
        dfdq    = [dfdq11, dfdq12; dfdq21, dfdq22];

        % Evaluate increment
        dq = -dfdq\f;

        % Update positions, velocities and accelerations, and pressures
        q_n1    = q_n1 + dq(1:7);
        qd_n1   = qdOldBow + 2.0/H * q_n1;
        qdd_n1  = qddOldBow + 4.0/H^2 * q_n1;

        p_n1    = p_n1 + dq(8:9);
        pd_n1   = pdOldBow + 2.0/H * p_n1; 

        % Check error
        error_e = norm(dq, 2);
        if abs(error_e) < abs(maxError)
            break;     
        end 

        % Increase number of iterations, warn if reached max iterations
        n_iter = n_iter+1;
        if (n_iter == maxIter-1)
            disp ('Reached max. num. of iterations');
        end
    end
    
    % Increase time and move to next integration time step
    t       = t + H;
    q       = q_n1;
    qd      = qd_n1;
    qdd     = qdd_n1;
    p       = p_n1;
    F       = (p(2) - p(1))*SYS.A - SYS.c*qd(7);
    pd      = pd_n1;
    lambda  = lambda_n1;
    
    % Need to project velocities and accelerations
    W           = MM + H/2.0 * C;
    ProjLead    = W + H^2/4.0*Jac'*alpha*Jac;
    
    for j=1:nProjs
        ProjRHS     = [W*qd, ...
                        W*qdd - H^2/4.0*Jac'*alpha*Jacdq];
        solProj     = ProjLead\ProjRHS;

        qd          = solProj(:,1);
        qdd         = solProj(:,2);
    end
    
end

end