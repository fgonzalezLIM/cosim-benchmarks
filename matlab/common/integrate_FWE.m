% Semi-implicit forward Euler integration formula
%
% Input
%  x:           Variables at time t_n
%  xd:          First derivatives at time t_n
%  xdd:         Second derivatives at time t_n
%  iPAR:        Integration parameters, including 
%   .H              step-size (s)
%
% Output
%  xn:          Variables at time t_n+1
%  xdn:         First derivatives at time t_n+1

function [xn, xdn] = integrate_FWE(x, xd, xdd, iPAR)

    xdn = xd + iPAR.H*xdd;
    xn  = x + iPAR.H*xdn;

end