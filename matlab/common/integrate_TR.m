% Implicit trapezoidal rule integration formula
%
% Input
%  x:           Variables at time t_n
%  xd:          First derivatives at time t_n
%  xdd:         Second derivatives at time t_n
%  SYS:         System definition structure - required to evaluate acceleration
%  iPAR:        Integration parameters, including 
%   .H              step-size (s)
%   .maxError       norm of maximum admissible increase in x after correction
%   .maxIter        maximum number of corrector iterations
%  evalAccFun:  Function to evaluate accelerations - problem dependent
%
% Output
%  xn:          Variables at time t_n+1
%  xdn:         First derivatives at time t_n+1

function [xn, xdn] = integrate_TR(x, xd, xdd, iPAR, SYS, evalAccFun)

    % Predictor
    xn     = x + iPAR.H * xd + iPAR.H^2/2.0*xdd;
    xdn    = xd + iPAR.H*xdd;
    
    % Corrector
    n_iter = 0;
    while n_iter <= iPAR.maxIter
        
        % Store for comparison
        xn_old = xn;  
        
        % Evaluate accelerations and integrate
        xddn = evalAccFun(SYS, xn, xdn);
        xn  = x + iPAR.H * xd + iPAR.H^2/4.0 * (xdd + xddn);
        xdn = xd + iPAR.H/2.0*(xdd + xddn);
        
        % Check error
        error_e = norm(xn - xn_old, 2);
        if abs(error_e) < abs(iPAR.maxError)
            break;     
        end 
        
        % Increase number of iterations
        n_iter = n_iter+1;
        if (n_iter == iPAR.maxIter-1)
            disp ('Reached max. num. of iterations');
        end      
    end
end