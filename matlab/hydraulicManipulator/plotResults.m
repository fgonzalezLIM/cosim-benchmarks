function indexPlots = plotResults(STORE, R_mono, indexPlots)

% ______________________________________________________________________________ Positions
indexPlots = indexPlots+1;
figure(indexPlots)
hold on
set(indexPlots, 'name', 'Actuator length s');
plot(R_mono.t,R_mono.pos(7,:), 'k');
plot(STORE.t,STORE.s(:,1), '--b');
xlabel('Time (s)');
ylabel('Actuator length (m)');
legend('mono', 'cosim');
hold off

% _____________________________________________________________________________ Velocities
indexPlots = indexPlots+1;
figure(indexPlots)
hold on
set(indexPlots, 'name', 'Actuator rate sd');
plot(R_mono.t,R_mono.vel(7,:), 'k');
plot(STORE.t,STORE.sd(:,1), '--b');
xlabel('Time (s)');
ylabel('Actuator rate (m/s)');
legend('mono', 'cosim');
hold off


end