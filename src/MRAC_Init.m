%% MRAC_Init.m  –  Full MRAC (replaces PD) for bipedal walking with payload
%  CORRECTED: reduced adaptation gains for stability

%% ── 1. Physical base parameters
g   = 9.81;
zc  = 0.90;
M0  = 10.0;

%% -- 2. Reference Model (STABILIZED)
k_p_ref = 100; 
k_d_ref = 20;

mrac.Am = [0,                           1;
          (g/zc - k_p_ref),            -k_d_ref]; 

mrac.Bm = [0; k_p_ref];
mrac.Cm = eye(2);
mrac.Dm = [0; 0];

%% ── 3. Lyapunov matrix P
Q_lyap  = eye(2);
mrac.P  = lyap(mrac.Am', Q_lyap);

%% ── 4. Adaptation gain – REDUCED for stability
mrac.Gamma = diag([0.05, 0.02, 0.01]);   % reduced 10x from original

%% ── 5. Robustness modifiers
mrac.sigma     = 0.05;    % increased sigma for more robustness
mrac.dead_zone = 5e-3;    % wider dead zone

%% ── 6. Initial parameter estimates
mrac.theta0_x = [0; 0; 0];
mrac.theta0_y = [0; 0; 0];

%% ── 7. Output saturation – REDUCED
mrac.u_mrac_max = 5.0;   % [N·m]

%% ── 8. Report
fprintf('==================================================\n');
fprintf('  MRAC_Init complete  (reduced gains for stability)\n');
eigs_P = eig(mrac.P);
if all(eigs_P > 0)
    fprintf('  Lyapunov P : positive-definite OK\n');
else
    warning('MRAC_Init: P not positive-definite!');
end
fprintf('==================================================\n');