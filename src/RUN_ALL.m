%% RUN_ALL.m  –  Master startup script
%  Run this ONE file in MATLAB before opening Simulink.
%  It initialises every workspace variable the model needs.
%  Order matters: do not rearrange.

clc; clear; close all;
fprintf('\n========= Bipedal Walker – Full Initialisation =========\n');

%% STEP 1 – MPC + State-Space model
fprintf('[1/4] Building LIPM state-space and MPC controller...\n');
run('StateSpace.m');

%% STEP 2 – ZMP / CoM reference trajectory
fprintf('[2/4] Generating ZMP staircase reference...\n');
run('ref.m');

%% STEP 3 – Gait scheduler (joint angle trajectories)
fprintf('[3/4] Building gait joint references...\n');
run('GaitScheduler.m');

%% STEP 4 – Payload model (water tank ramp)
fprintf('[4/4a] Building payload ramp signal...\n');
run('PayloadModel.m');

%% STEP 4b – MRAC parameters
fprintf('[4/4b] Initialising MRAC (replaces PD)...\n');
run('MRAC_Init.m');

fprintf('\n========= All workspace variables ready =========\n');
fprintf('Variables in workspace:\n');
fprintf('  mpc_controller  – MPC object (from StateSpace.m)\n');
fprintf('  ref_ts          – ZMP/CoM reference timeseries\n');
fprintf('  joint_ref       – 8-joint angle timeseries\n');
fprintf('  payload_ts      – water-tank mass ramp\n');
fprintf('  mrac            – MRAC struct (Am,Bm,P,Gamma,theta0_x/y,...)\n');
fprintf('\nNow open BipedalWalkCopy.slx and press Run.\n');
fprintf('=================================================\n');
