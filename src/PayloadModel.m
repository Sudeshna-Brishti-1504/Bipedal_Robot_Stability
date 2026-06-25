%% PayloadModel.m  –  Time-varying water-tank payload
%  Outputs a Simulink signal: payload mass m(t) as a ramp
%  Units: kg.  Attach to a "From Workspace" block named payload_ts.

Ts         = 0.05;
t          = (0:Ts:10)';
N          = length(t);

m0         = 0.0;    % initial payload mass  [kg]   – empty tank
m_final    = 5.0;    % final payload mass    [kg]   – full tank
t_fill     = 8.0;    % time over which tank fills  [s]

% Linear ramp, clamped once full
m_payload  = min(m0 + (m_final - m0) .* max(t - 0.5, 0) ./ t_fill, m_final);

payload_ts = timeseries(m_payload, t);

figure('Name','Payload mass vs time','Color','w');
plot(t, m_payload, 'b', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Payload mass (kg)');
title('Water-tank payload ramp');

fprintf('PayloadModel: m0=%.1f kg → m_final=%.1f kg over %.1f s\n', ...
        m0, m_final, t_fill);
