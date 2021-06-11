% This file defines a class that represents a subsystem of the linear oscillator.

%   |         |----|             
%   |--/\/\/--| m1 |----/\/\/----o
%   | k1, c1  |----|   k_c, c_c  

% Input: 	position, velocity, and force from the other subsystem [x, xd, f]
% Output: 	position, velocity, and force from this subsystem [x, xd, f]

% Input and output are generic and include the position and velocity of the mass and 
% the force of the spring that connects both subsystems.
% See constructor documentation below.

% Depending on the object configuration, the block receives force or positions as input. 
% This is also the case with the outputs.

classdef mass_ss < CosimUnit
    
    % _________________________________________________________________________ PROPERTIES
    properties
        Y_;         % Outputs [x, xd, f]
    end
    
    % _______________________________________________________________ PROPERTIES (PRIVATE)
    properties (Access=private)
        t;          % Internal time
        OPT;        % Solution options
        
        idx;        % Subsystem index
        inputType;  % Type of input received (force 1 or displacement 2)
        outputType; % Type of output received (force 1 or displacement 2)
        
        x; xd; xdd; % Position, velocity, acceleration
        s; sd;      % Position and velocity of external subsystem
        fc;         % Coupling force from external subsystem
        SYS;        % Physical properties
        
        % Internally calculated forces
        f_sp;       % Wall force (spring)
        f_dp;       % Wall force (damper)
        f_spc;      % Coupling spring
        f_dpc;      % Coupling damper
        f;          % Overall coupling force
        
    end
    
    % ____________________________________________________________________________ METHODS
    methods
        
        % ____________________________________________________________________ Constructor
        function b = mass_ss(name, idx, iType, oType, OPT)
            
            % Parameters:
            %   name:           The name of the subsystem
            %   idx:            Index of the subsystem
            %   iType:          Type of input, can be "F" (force) or "S" (displacement)
            %   oType:          Type of output, can be "F" (force) or "S" (displacement)
            %   OPT:            Simulation options, including
            %       .integrator;    Integrator type
            %       .scenario:  	Scenario number
            
            % Call constructor of base class
            b@CosimUnit(name);
            
            % Index (first or second subsystem)
            if ((idx == 1) || (idx == 2))
                b.idx  = idx;
            else
                error ("Wrong subsystem index, must be 1 or 2.");
            end
            
            % Input: force or displacement
            if (iType == 'S')
                b.inputType = 2;
            elseif (iType == 'F')
                b.inputType = 1;
            else
                error ("Wrong input type, must be F or S.");
            end
            
            % Output: force or displacement
            if (oType == 'S')
                b.outputType = 2;
            elseif (oType == 'F')
                b.outputType = 1;
            else
                error ("Wrong output type, must be F or S.");
            end
            
            % Mass block properties
            props       = getOscillatorProperties(OPT.scenario);
            b.OPT       = OPT;
            b.SYS.m     = props.m(idx);
            b.SYS.k     = props.k(idx);
            b.SYS.c     = props.c(idx);
            b.SYS.x0    = props.x0(idx);
            b.SYS.xd0   = props.xd0(idx);
            b.SYS.kc    = props.kc;     % Coupling stiffness
            b.SYS.cc    = props.cc;     % Coupling damping

            % Stablish initial state
            b.x         = b.SYS.x0;
            b.xd        = b.SYS.xd0;
            
            % Set initial forces to zero
            b.f_spc     = 0.0;
            b.f_dpc     = 0.0;
            b.f_sp      = 0.0;
            b.f_dp      = 0.0;
            b.f         = 0.0;
            
        end
        
        % _____________________________________________________________________ Initialize 
        function b = initialize(b,t)
            
            str = "Initializing mass subsystem " + b.idx;   disp(str);
            
            % Set time to externally set time
            b.t = t;
            
            % If output is force, evaluate
            if (b.outputType == 1) 
                evaluateSpringForces(b, b.x, b.xd);
                b.f = b.f_spc + b.f_dpc; 
            end           
        end
        
        % _________________________________________________________________________ doStep
        function b = doStep(b, finalT)
            
            % Integration step-size
            dt = b.OPT.H;
            
            % Take steps until t goes beyond finalT
            while (finalT - b.t > dt/2)
                
                % Evaluate accelerations
                b.evaluateMassAcceleration(b.x, b.xd); 
                
                % Integrate
                if(b.OPT.integrator == 1)
                    [b.x, b.xd] = integrate_FWE(b.x, b.xd, b.xdd, b.OPT);
                elseif(b.OPT.integrator == 2)
                    [b.x, b.xd] = integrate_TR(b.x, b.xd, b.xdd, b.OPT, b, @evaluateMassAcceleration);
                end
                
                % Increase time
                b.t   = b.t + dt;
                
                % If output is force, evaluate
                if (b.outputType == 1) 
                    evaluateSpringForces(b, b.x, b.xd);
                    b.f = b.f_spc + b.f_dpc; 
                end
            end
                   
        end
        
        % ______________________________________________________________________ Terminate 
        function b = terminate(b, t)
            str = "Terminating mass block " + b.idx + ", time: " + t; 
            disp(str);
        end
        
        % ___________________________________________________________________ Read outputs
        function y = readOutputs(b)
            % Prepare outputs before they are sent to the other subsystem
            b.Y_ = [b.x; b.xd; b.f];
            y = b.Y_;
        end
        
        % _____________________________________________________________________ Set inputs
        function setInputs(b, u)
            % Inputs from other subsystem
            if (b.inputType == 2)       % Receive position and velocity
                b.s     = u(1);
                b.sd    = u(2);
                b.fc    = NaN;
            elseif (b.inputType == 1)   % Receive force
                b.s     = NaN;
                b.sd    = NaN;
                b.fc    = u(3);
            end
        end
        
        % __________________________________________________ Evaluate spring/damper forces
        function evaluateSpringForces(b, x, xd)
                
            % Coupling spring and damper - evaluate only if input is position/velocity
            if (b.inputType == 2)
                b.f_spc  = -(b.SYS.kc*(x - b.s));
                b.f_dpc  = -(b.SYS.cc*(xd - b.sd));
            else
                b.f_spc = b.fc;
                b.f_dpc = 0;
            end

            % Spring and damper betweeen mass and wall
            b.f_sp = - b.SYS.k * x;
            b.f_dp = - b.SYS.c * xd;
        end
        
        % _________________________________________________________ Evaluate accelerations
        function xdd = evaluateMassAcceleration(b, x, xd)
            b.evaluateSpringForces(x, xd);
            LEAD    = b.SYS.m;
            RHS     = b.f_spc + b.f_dpc + b.f_sp + b.f_dp;
            b.xdd   = LEAD\RHS;
            xdd     = b.xdd;
        end
        
    end
    
end