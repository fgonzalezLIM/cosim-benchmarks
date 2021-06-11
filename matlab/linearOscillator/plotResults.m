function indexPlots = plotResults(STORE, R_an, R_mono, indexPlots)

% ______________________________________________________________________________ Positions
indexPlots = indexPlots+1;
figure(indexPlots)
hold on
set(indexPlots, 'name', 'Position x1');
plot(R_an.t,R_an.x(:,1), 'k');
plot(R_mono.t,R_mono.x(:,1), '--r');
plot(STORE.t,STORE.x(:,1), ':b');
xlabel('Time (s)');
ylabel('Position (m)');
legend('ref', 'mono', 'cosim');
hold off

indexPlots = indexPlots+1;
figure(indexPlots)
hold on
set(indexPlots, 'name', 'Position x2');
plot(R_an.t,R_an.x(:,2), 'k');
plot(R_mono.t,R_mono.x(:,2), '--r');
plot(STORE.t,STORE.x(:,2), ':b');
xlabel('Time (s)');
ylabel('Position (m)');
legend('ref', 'mono', 'cosim');
hold off

% _____________________________________________________________________________ Velocities
indexPlots = indexPlots+1;
figure(indexPlots)
hold on
set(indexPlots, 'name', 'Velocity x1');
plot(R_an.t,R_an.xd(:,1), 'k');
plot(R_mono.t,R_mono.xd(:,1), '--r');
plot(STORE.t,STORE.xd(:,1), ':b');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('ref', 'mono', 'cosim');
hold off

indexPlots = indexPlots+1;
figure(indexPlots)
hold on
set(indexPlots, 'name', 'Velocity x2');
plot(R_an.t,R_an.xd(:,2), 'k');
plot(R_mono.t,R_mono.xd(:,2), '--r');
plot(STORE.t,STORE.xd(:,2), ':b');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('ref', 'mono', 'cosim');
hold off

% _________________________________________________________________________________ Energy
indexPlots = indexPlots+1;
figure(indexPlots)
hold on
set(indexPlots, 'name', 'Mechanical energy');
plot(R_an.t,R_an.mechEn, 'k');
plot(R_mono.t,R_mono.mechEn, '--r');
plot(STORE.t,STORE.mechEn, ':b');
xlabel('Time (s)');
ylabel('Mechanical Energy (J)');
legend('ref', 'mono', 'cosim');
hold off

end