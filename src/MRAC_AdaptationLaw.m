%% MRAC_AdaptationLaw.m
%
%  Paste the function body below into a Simulink MATLAB Function block.
%  Block signature:
%    Inputs  : x_com  (2×1) [pos; vel] from plant (one axis)
%              x_m    (2×1) [pos; vel] from reference model (one axis)
%              theta  (3×1) current parameter estimate
%              zmp_ref (scalar) ZMP reference for this axis
%    Outputs : theta_dot (3×1) parameter update rate
%              u_mrac    (scalar) adaptive feedforward signal
%
%  Use two instances: one for sagittal (x), one for lateral (y).
%  Integrate theta_dot with a continuous-time Integrator block (IC = theta0).

function [theta_dot, u_mrac] = mrac_adaptation(x_com, x_m, theta, zmp_ref, P, Gamma, sigma, dead_zone, u_max)
%#codegen

%% Error between plant and reference model
e  = x_com - x_m;   % [2×1]:  [pos_err; vel_err]

%% Regressor vector  φ(x) = [pos; vel; 1]
phi = [x_com(1); x_com(2); 1];   % [3×1]

%% Adaptive feedforward
u_mrac_raw = theta' * phi;        % scalar
u_mrac     = max(min(u_mrac_raw, u_max), -u_max);   % saturate

%% MIT/Lyapunov adaptation law with sigma-modification and dead-zone
%   θ̂̇ = -Γ · φ · (Bᵀ P e) - σ·Γ·θ̂
%   B for the reference model is [0; -g/zc], so Bᵀ P e is a scalar.
%   We absorb the sign into the adaptation direction.
BtPe = (P(2,1)*e(1) + P(2,2)*e(2)) * (-1);   % Bᵀ = [0, -g/zc], normalise to ≈1 inside Γ

if norm(e) > dead_zone
    theta_dot = -Gamma * phi * BtPe - sigma * (Gamma * theta);
else
    theta_dot = zeros(3,1);   % freeze adaptation in dead-zone
end

end

%% ─────────────────────────────────────────────────────────────────────────
%  SIMULINK WIRING GUIDE
%  ─────────────────────────────────────────────────────────────────────────
%  1. Add a MATLAB Function block, paste the function above.
%  2. Use a Constant block to pass mrac.P, mrac.Gamma, mrac.sigma,
%     mrac.dead_zone, mrac.u_mrac_max as parameters (or use workspace vars).
%  3. Add an Integrator block for theta (IC = mrac.theta0_x).
%     Connect theta_dot → Integrator → theta (feedback).
%  4. Run two parallel instances: one for X axis, one for Y axis.
%  5. Outputs:
%       u_mrac_x → sum with u_PD_x before the sagittal joint torque input
%       u_mrac_y → sum with u_PD_y before the lateral  joint torque input
%  6. Reference model: implement mrac.Am / mrac.Bm as a State-Space block,
%     driven by zmp_ref (from ref.m timeseries).
%  ─────────────────────────────────────────────────────────────────────────
