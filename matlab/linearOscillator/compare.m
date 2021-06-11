clear variables
close all

% __________________________________________________ Extend path to find library functions
addpath("../common");

% ________________________________________________________________________ Define scenario

scenario    = 0;        % Simulation case to be solved - available: 0, 1.
finalT      = 10.0;     % Final time of motion (s)
Han          = 0.01;    % Sampling interval for results storage (s)
Hmono        = 0.001;   % Sampling interval for results storage (s)

% Options structure for the integration
OPTIONS.saveEvery   =   10;     % Storage rate
OPTIONS.integrator  =   1;      % 1: Forward-Euler

% _______________________________________________________________________________ Solution
SYS = getOscillatorProperties(scenario);
RES_an      = simulate_analytic(Han, finalT, SYS);
RES_mono    = simulate_monolithic(Hmono, finalT, OPTIONS, SYS);


% ___________________________________________________________________________________ Plot
figure(1)
hold on
set(1, 'name', 'Positions');
plot(RES_an.t,RES_an.x(:,1), 'r');
plot(RES_an.t,RES_an.x(:,2), 'r');
plot(RES_mono.t,RES_mono.x(:,1), '--k');
plot(RES_mono.t,RES_mono.x(:,2), '--b');
xlabel('Time (s)');
ylabel('Position (m)');
legend('x_1ref', 'x_2ref', 'x_1', 'x_2');
hold off

figure(2)
hold on
set(2, 'name', 'Velocities');
plot(RES_an.t,RES_an.xd(:,1), 'r');
plot(RES_an.t,RES_an.xd(:,2), 'r');
plot(RES_mono.t,RES_mono.xd(:,1), '--k');
plot(RES_mono.t,RES_mono.xd(:,2), '--b');
xlabel('Time (s)');
ylabel('Velocities (m/s)');
legend('xd_1ref', 'xd_2ref', 'xd_1', 'xd_2');
hold off


% ___________________________________________________________________ Restore initial path
rmpath("../common");