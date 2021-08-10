% This file defines a class that represents a double pendulum system that is controlled 
% with a hydraulic actuator.
%
% The system is modelled using the angles of its rods w.r.t. the horizontal (x) axis as
% generalized coordinates

% Input: 	force OR pressures in the cylinder
% Output: 	displacement and velocity of the cylinder

classdef manipulator_ss < CosimUnit

    % _________________________________________________________________________ PROPERTIES
    properties
        Y_;         % Outputs (piston length and rate)
    end

    % _______________________________________________________________ PROPERTIES (PRIVATE)
    properties (Access=private)
        t;          % Internal time
        OPT;        % Solution options

        th; thd; thdd;  % Position, velocity, accel. (vars. are joint angles)
        s; sd;          % Position and velocity of actuator

        uIsF;       % 1 if input is force, 0 if it is pressures
        fhyd;       % Actuation force
        p;          % Hydraulic pressures (only required if input = p)
    
        SYS;        % Physical properties
        STORE;      % Storage
       
    end

    % ____________________________________________________________________________ METHODS
    methods

        % ____________________________________________________________________ Constructor
        function b = manipulator_ss(name, iType, OPT)
            
            % Parameters:
            %   name:           The name of the subsystem
            %   iType:          Type of input, can be "F" (force) or "P" (pressures)
            %   OPT:            Simulation options, including
            %       .H              Step-size
            %       .integrator     Integrator type
    
            % Call constructor of base class
            b@CosimUnit(name);
            
            % Input: force or pressures
            if (iType == 'P')
                b.uIsF = 0;
            elseif (iType == 'F')
                b.uIsF = 1;
            else
                error ("Wrong input type, must be F or P.");
            end

            % Set physical properties
            b.SYS = getManipulatorProperties(-1);
            b.OPT = OPT;

            % Initial angles and rates
            b.th  = [b.SYS.th; b.SYS.th2];
            b.thd = zeros(2,1);

            % Initial actuator displacement and velocity
            [b.s, b.sd, ~] = findSfromAngles(b.th, b.thd, b.SYS);

        end

        % _____________________________________________________________________ Initialize 
        function b = initialize(b,t)
            
            str = "Initializing mechanics";    disp(str);

            % Set time to externally set time
            b.t = t;
            
            % Storage
            dt = b.OPT.H;
            b.STORE.npoints     = round(max(b.tEnd/(dt*b.storeEvery), 1));
            b.STORE.i           = b.storeEvery-1;   % Storage counter

            np = b.STORE.npoints;
            b.STORE.t   = zeros(1, np);
            b.STORE.s   = zeros(1, np);
            b.STORE.sd  = zeros(1, np);
            b.STORE.th  = zeros(2, np);
            b.STORE.thd = zeros(2, np);
            b.STORE.thdd= zeros(2, np);
            b.STORE.storeIdx = 0;

        end

        % _________________________________________________________________________ doStep
        function b = doStep(b, finalT)
            
            % Integration step-size
            dt = b.OPT.H;

            % Take steps until t goes beyond finalT
            while (finalT - b.t > dt/2)
                
                % Evaluate accelerations
                b.evaluateAcceleration(b.th, b.thd); 

                % Store
                b.STORE.i = b.STORE.i+1;
                if b.STORE.i == b.storeEvery
                    b.STORE.storeIdx = b.STORE.storeIdx + 1;
                    si = b.STORE.storeIdx;
                    b.STORE.t(si)       = b.t;
                    b.STORE.th(:,si)    = b.th;  
                    b.STORE.thd(:,si)   = b.thd; 
                    b.STORE.thdd(:,si)  = b.thdd;
                    b.STORE.s(si)       = b.s;  
                    b.STORE.sd(si)      = b.sd;
                    b.STORE.F(si)       = b.fhyd;
                    b.STORE.i = 0;
                end
                
                % Integrate
                if(b.OPT.integrator == 1)
                    [b.th, b.thd] = integrate_FWE(b.th, b.thd, b.thdd, b.OPT);
                elseif(b.OPT.integrator == 2)
                    [b.th, b.thd] = integrate_TR(b.th, b.thd, b.thdd, b.OPT,...
                        b, @evaluateAcceleration);
                end
 
                % Increase time
                b.t   = b.t + b.OPT.H;
            end
        end

        % ______________________________________________________________________ Terminate 
        function b = terminate(b, t)
            str = "Terminating mechanics.";    disp(str);
        end

        % ___________________________________________________________________ Read outputs
        function y = readOutputs(b)

            % Re-evaluate displacements
            [b.s, b.sd, ~] = findSfromAngles(b.th, b.thd, b.SYS);
            b.Y_ = [b.s;b.sd];

            y = b.Y_;
        end
        
        % _____________________________________________________________________ Set inputs
        function setInputs(b, u)
            
            if (b.uIsF == 1)
                % Hydraulic force
                b.fhyd     = u(3);
            elseif (b.uIsF == 0)
                % Pressures
                b.p         = [u(1);u(2)];
            end
        end

        % _______________________________________________________ Evaluate effective force
        function feff0 = evalEffectiveForce(b)
            % Returns the effective force of the mechanical system 
            % along the direction of the hydraulic actuator

            % In direct co-simulation, this function is used only 
            % to initialize the hydraulics
            MM          = evalMassMatrix(b.th, b.SYS);
            fg          = evalGravForces(b.th, b.SYS);
            fc          = evalVelDepForces(b.th, b.thd, b.SYS);

            % Eval Jacobian matrix of cylinder direction
            [~, ~, J]   = findSfromAngles(b.th, b.thd, b.SYS);

            meff        = (J*MM^(-1)*J')^(-1);
            feff0       = meff * J * MM^(-1) * (fg-fc);

        end
        
        % _________________________________________________________ Evaluate accelerations
        function thdd = evaluateAcceleration(b, th, thd)
                
                % Update dynamic terms
                M   = evalMassMatrix(th, b.SYS);
                fc  = evalVelDepForces(th, thd, b.SYS);
                fg  = evalGravForces(th, b.SYS);
                [b.s, b.sd, Ai] = findSfromAngles(th, thd, b.SYS);
                
                % Hydraulic force
                % If pressures were used as input, the hydraulic force
                % needs to be evaluated
                if (b.uIsF == 0)
                    b.fhyd = (b.p(2) - b.p(1))*b.SYS.A - b.SYS.c*b.sd;
                end
                fh  = Ai'*b.fhyd;

                % Evaluate accelerations
                b.thdd = M\(fg-fc + fh);
                thdd = b.thdd;
        end
        
        % ______________________________________________________________________ Accessors
        function STORE = getSTORE(b); STORE = b.STORE; end
        function setStepSize(b, dt); b.iPAR.dt = dt; end

    end

end