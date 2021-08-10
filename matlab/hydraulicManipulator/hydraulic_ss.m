% This file defines a class that represents the hydraulics actuation of a single-actuator 
% 2-d.o.f crane.

% Input: 	displacement and velocity of hydraulic actuator
% Output: 	hydraulic force OR hydraulic pressures in the cylinder

classdef hydraulic_ss < CosimUnit
    
	% _________________________________________________________________________ PROPERTIES
    properties
        Y_;         % Outputs (force OR pressures)
    end
    
    % _______________________________________________________________ PROPERTIES (PRIVATE)
    properties (Access=private)
        t;          % Internal time
        OPT;        % Integration parameters
        
        yIsF;       % 1 if return force, 0 if return pressures
        p; pd;      % Hydraulic pressures and their derivatives
        kappa;      % Valve command
        SYS;        % Physical properties
        
        s; sd;      % Inputs
        Fhyd;       % Output force
        feff0;      % Initial force exerted by the cylinder
        
        STORE;      % Storage
    end
    
    % ____________________________________________________________________________ METHODS
    methods
        
        % ____________________________________________________________________ Constructor
        function b = hydraulic_ss(name, oType, OPT)
            
            % Parameters:
            %   name:           The name of the subsystem
            %   oType:          Type of output, can be "F" (force) or "P" (pressures)
            %   OPT:            Simulation options, including
            %       .H              Step-size
            %       .integrator     Integrator type
            %       .scenario       0-steps, 1-sinusoidal
            
            % Call constructor of base class
            b@CosimUnit(name);
                        
            % Output: force or pressures
            if (oType == 'P')
                b.yIsF = 0;
            elseif (oType == 'F')
                b.yIsF = 1;
            else
                error ("Wrong output type, must be F or P.");
            end
            
            % Set physical properties
            b.SYS = getManipulatorProperties(OPT.scenario);
            b.OPT = OPT;
            
        end
        
        % _____________________________________________________________________ Initialize 
        function b = initialize(b,t)
            
            str = "Initializing hydraulics";    disp(str);

            % Make sure that inputs have been received at least once
            assert(size(b.s,1)==1, "Wrong size in b.s. - Non-initialized inputs?");
            assert(size(b.sd,1)==1, "Wrong size in b.sd. - Non-initialized inputs?");
            
            % Set time to externally set time
            b.t = t;
            
            % Storage
            dt = b.OPT.H;
            b.STORE.npoints     = max(b.tEnd/(dt*b.storeEvery), 1);
            b.STORE.i           = b.storeEvery-1;   % Storage counter
            
            np = b.STORE.npoints;
            b.STORE.t       = zeros(1, np);
            b.STORE.kappa   = zeros(1, np);
            b.STORE.s       = zeros(1, np);
            b.STORE.sd      = zeros(1, np);
            b.STORE.p       = zeros(2, np);
            b.STORE.pd      = zeros(2, np);
            b.STORE.F       = zeros(1, np);
            b.STORE.storeIdx = 0;
            
            % Initial length of actuator
            b.SYS.s0  = b.s;

            % We need to determine the initial pressures
            b.p     = [3.3e6;4.4e6];        % Initial guess
            b.kappa = 0.5;                  % Spool displacement (valve)
            [b.p, b.kappa, ~] = evalIniPressure(b.p, b.kappa, b.s, ...
                b.SYS, b.feff0, 3);

            % Store initial value of kappa
            b.SYS.kappa0 = b.kappa;

            % Evaluate initial pressure rates (should be zero)
            [h1,h2] = evalPressureRates(b.s,b.sd,b.p,b.kappa,b.SYS);
            b.pd = [h1;h2];

            % Evaluate initial force
            b.Fhyd = (b.p(2) - b.p(1))*b.SYS.A - b.SYS.c*b.sd;
            
        end
        
        % _________________________________________________________________________ doStep
        function b = doStep(b, finalT)
            
            % Take steps until t goes beyond finalT
            while (b.t - finalT < b.OPT.H/2)

                % Update kappa
                b.kappa       = updateKappa(b.t, b.SYS);

                % Evaluate pressure rates
                [h1,h2] = evalPressureRates(b.s,b.sd,b.p,b.kappa,b.SYS);
                b.pd = [h1;h2];

                % Store
                b.STORE.i = b.STORE.i+1;
                if b.STORE.i == b.storeEvery
                    b.STORE.storeIdx = b.STORE.storeIdx + 1;
                    si = b.STORE.storeIdx;
                    b.STORE.t(si)     = b.t;
                    b.STORE.kappa(si) = b.kappa;  
                    b.STORE.s(si)     = b.s; 
                    b.STORE.sd(si)    = b.sd;
                    b.STORE.p(:,si)   = b.p;
                    b.STORE.pd(:,si)  = b.pd;
                    b.STORE.F(:,si)   = b.Fhyd;
                    b.STORE.i = 0;
                end

                % Integrate
                if(b.OPT.integrator == 1)
                    b.p   = b.p + b.OPT.H * b.pd;
                elseif(b.OPT.integrator == 2)
                    error("Method not implemented yet");
                end

                % Increase time
                b.t   = b.t + b.OPT.H;
            end

        end
        
        % ______________________________________________________________________ Terminate 
        function b = terminate(b, t)
            str = "Terminating hydraulics.";    disp(str);
        end
        
        % ___________________________________________________________________ Read outputs
        function y = readOutputs(b)

            % Evaluate force
            b.Fhyd = (b.p(2) - b.p(1))*b.SYS.A - b.SYS.c*b.sd;
            
            if (b.yIsF == 1)
                % Return the force and zeros instead of pressures
                b.Y_ = [0.0; 0.0; b.Fhyd];
            elseif (b.yIsF == 0)
                % Return pressures and a zero instead of the force
                b.Y_ = [b.p(1); b.p(2); 0.0];
            end

            y = b.Y_;
        end
        
        % _____________________________________________________________________ Set inputs
        function setInputs(b, u)
            % Position and velocity of external subsystem
            b.s     = u(1);
            b.sd    = u(2);
        end

		% ____________________________________________________________ Set effective force
        function setEffectiveForce(b,feff0)
            % Sets the effective force coming from the mechanics.
            % Used in initialization only
            b.feff0 = feff0;
        end
        
        % ______________________________________________________________________ Accessors
        function STORE = getSTORE(b); STORE = b.STORE; end
        function setStepSize(b, dt); b.iPAR.dt = dt; end
        
    end
    
end