%% MRAC_Monitor.m  –  Post-simulation analysis
%  Run after simulation. Expects workspace variables logged from Simulink:
%    tout        – time vector
%    e_x, e_y    – CoM tracking errors (sagittal, lateral)  [m]
%    theta_x     – 3×N parameter estimates (sagittal)
%    theta_y     – 3×N parameter estimates (lateral)
%    u_mrac_x, u_mrac_y  – adaptive feedforward outputs
%    m_payload   – payload mass timeseries
%    zmp_x, zmp_y        – actual ZMP positions (logged from MPC output)

%% ── Figure 1: Tracking error ──────────────────────────────────────────────
figure('Name','MRAC Tracking Performance','Color','w','Position',[100 100 900 500]);

subplot(2,2,1);
plot(tout, e_x*100, 'b', 'LineWidth', 1.2); hold on;
plot(tout, zeros(size(tout)), 'k--', 'LineWidth', 0.8);
xlabel('Time (s)'); ylabel('Error (cm)');
title('Sagittal CoM error e_x'); grid on;
legend('e_x','zero','Location','best');

subplot(2,2,2);
plot(tout, e_y*100, 'r', 'LineWidth', 1.2); hold on;
plot(tout, zeros(size(tout)), 'k--', 'LineWidth', 0.8);
xlabel('Time (s)'); ylabel('Error (cm)');
title('Lateral CoM error e_y'); grid on;
legend('e_y','zero','Location','best');

subplot(2,2,3);
plot(tout, theta_x', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('\theta');
title('Adapted parameters \theta_x (sagittal)'); grid on;
legend('\theta_{pos}','\theta_{vel}','\theta_{bias}','Location','best');

subplot(2,2,4);
plot(tout, theta_y', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('\theta');
title('Adapted parameters \theta_y (lateral)'); grid on;
legend('\theta_{pos}','\theta_{vel}','\theta_{bias}','Location','best');

sgtitle('MRAC + Payload: Tracking & Adaptation');

%% ── Figure 2: Control effort decomposition ───────────────────────────────
figure('Name','Control Effort Breakdown','Color','w','Position',[100 650 900 300]);
subplot(1,2,1);
plot(tout, u_mrac_x, 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Force (N)');
title('Adaptive FF  u_{mrac,x}'); grid on;

subplot(1,2,2);
yyaxis left;  plot(tout, u_mrac_y, 'r', 'LineWidth', 1.2); ylabel('Force (N)');
yyaxis right; plot(tout, m_payload,'k--','LineWidth',1.2);  ylabel('Payload (kg)');
xlabel('Time (s)'); title('u_{mrac,y} vs payload'); grid on;
legend('u_{mrac,y}','m(t)','Location','best');

%% ── Summary statistics ───────────────────────────────────────────────────
fprintf('\n========= MRAC Performance Summary =========\n');
fprintf('  RMS error sagittal  : %.4f m  (%.2f cm)\n', rms(e_x), rms(e_x)*100);
fprintf('  RMS error lateral   : %.4f m  (%.2f cm)\n', rms(e_y), rms(e_y)*100);
fprintf('  Peak |e_x|          : %.4f m\n', max(abs(e_x)));
fprintf('  Peak |e_y|          : %.4f m\n', max(abs(e_y)));
fprintf('  Final theta_x       : [%.3f, %.3f, %.3f]\n', theta_x(end,1), theta_x(end,2), theta_x(end,3));
fprintf('  Final theta_y       : [%.3f, %.3f, %.3f]\n', theta_y(end,1), theta_y(end,2), theta_y(end,3));
fprintf('  Final payload mass  : %.2f kg\n', m_payload(end));
fprintf('=============================================\n');
