% Semi-implicit forward Euler integration formula
%
% Input
%  x:           Variables at time t_n
%  xd:          First derivatives at time t_n
%  xdd:         Second derivatives at time t_n
%  H:           Integration step-size (s)
%
% Output
%  xn:          Variables at time t_n+1
%  xdn:         First derivatives at time t_n+1

function [xn, xdn] = integrate_FWE(x, xd, xdd, H)

    xdn = xd + H*xdd;
    xn  = x + H*xdn;

end