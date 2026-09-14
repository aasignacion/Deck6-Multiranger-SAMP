function results = run_sweep_fan_deg_Deck6_env4()
%RUN_SWEEP_FAN_DEG_DECK6_ENV4 Sweeps the r_FL/r_FR mounting yaw angle
% (FAN_DEG in raycast_ranges6_obbs.m -- the hardware PLACEMENT angle,
% not the sensor's own FoV) across 15/22.5/30/45 deg, on Env4, baseline
% config (no RSL, no SMM), 0% dropout -- same scenario/speed as
% run_dropout_xy_trajectories_MR_vs_Deck6_baseline_env4.m, where Deck6
% at the real-hardware 22.5deg got 100% success (n=3). This checks
% whether that result is sensitive to the exact mounting angle, or
% whether nearby angles do just as well/better.
%
% Deck6 only -- MR has no FAN_DEG (no FL/FR beams), so it isn't part of
% this sweep.
%
% Requires in path: load_sdf_obbs, collision_sphere_obbs,
% raycast_ranges6_obbs, apply_range_noise, simulate_controller6_logmem,
% ctrl_wall_extend_ref_h_only_vl53l8cx6, compute_nav_metrics, wilson_ci.

clc;

% ====================== SCENARIO (matches the env4 baseline comparison) ======================
S = struct();
S.sdf_path = 'Env4_combined_trans.sdf';
S.init = struct('x',0.0,'y',0.0,'z',0.25,'yaw',0.0);
S.goal = struct('x',-0.5,'y',5.5,'z',0.25);
S.T = 500;

env = struct();
env.obbs = load_sdf_obbs(S.sdf_path);
env.max_range = 3.0;

simBase = struct();
simBase.dt = 0.01;
simBase.sensor_dt = 0.10;
simBase.T = S.T;
simBase.robot_radius = 0.08;
simBase.goal_tol = 0.15;
simBase.v_limits = [0.3 0.1 0.1];
simBase.w_limit  = 1.5;
simBase.z_limits = [0.05 5.0];
simBase.init = S.init;
simBase.goal = S.goal;
simBase.is_vertical = false;
simBase.exit_detect = struct();
simBase.bug_params = struct('debug_print', false, ...
    'enable_reflex_safety', false, 'enable_memory_ttc', false);   % baseline

% FAN_DEG_VALUES = [15, 22.5, 30, 45];
FAN_DEG_VALUES = [15, 17.5, 20, 22.5, 25, 27.5, 30, 32.5, 35, 37.5, 40, 42.5, 45];
N_TRIALS       = 1;
D_MIN_METRIC   = 0.12;

results = struct('fan_deg', {}, 'trial', {}, 'stop_reason', {}, 'success', {}, 'collision', {}, ...
    'pathEfficiency', {}, 'totalTime', {}, 'minClearance', {}, 'nCornerEpisodes', {});

for fi = 1:numel(FAN_DEG_VALUES)
    fanDeg = FAN_DEG_VALUES(fi);
    fprintf('\n=== Deck6 baseline, FAN_DEG=%.1f deg (0%% dropout, %d trials) ===\n', fanDeg, N_TRIALS);

    for trial = 1:N_TRIALS
        rng(fi*1000 + trial, 'twister');

        sim = simBase;
        sim.fan_deg = fanDeg;
        sim.sensor_noise = struct('sigma0', 0.005, 'sigma_slope', 0.01, ...
            'dropout_prob', 0.0, 'max_range', env.max_range);

        log = simulate_controller6_logmem(@ctrl_wall_extend_ref_h_only_vl53l8cx6, ...
            sprintf('Deck6 fan=%.1f trial=%d', fanDeg, trial), env, sim);

        m = compute_nav_metrics(log, D_MIN_METRIC);
        idx = numel(results) + 1;
        results(idx).fan_deg = fanDeg;
        results(idx).trial = trial;
        results(idx).stop_reason = char(log.stop_reason);
        results(idx).success = log.goal_reached || log.exited;
        results(idx).collision = log.collision;
        results(idx).pathEfficiency = m.pathEfficiency;
        results(idx).totalTime = m.totalTime;
        results(idx).minClearance = m.minClearance;
        results(idx).nCornerEpisodes = m.nCornerEpisodes;

        fprintf('  trial %2d: stop=%-16s success=%d pathEff=%.2f minClear=%.2fm totalTime=%.1fs\n', ...
            trial, results(idx).stop_reason, results(idx).success, m.pathEfficiency, m.minClearance, m.totalTime);
    end
end

% ====================== checkpoint ======================
matPath = fullfile(pwd, 'sweep_fan_deg_Deck6_env4_results.mat');
save(matPath, 'results', 'S', 'FAN_DEG_VALUES', 'N_TRIALS', 'D_MIN_METRIC');
fprintf('\nRaw results checkpointed to: %s\n', matPath);

% ====================== summary ======================
fprintf('\n=====================================================================\n');
fprintf('FAN_DEG sweep summary (n=%d trials each)\n', N_TRIALS);
fprintf('=====================================================================\n');
fanVals = [results.fan_deg];
for fi = 1:numel(FAN_DEG_VALUES)
    sel = (fanVals == FAN_DEG_VALUES(fi));
    n = sum(sel);
    nSucc = sum([results(sel).success]);
    nCrash = sum([results(sel).collision]);
    nTimeout = n - nSucc - nCrash;
    [lo, hi] = wilson_ci(nSucc, n);
    fprintf('  FAN_DEG=%5.1f: success=%d/%d (%.0f%%, 95%% CI [%.0f%%,%.0f%%])  crash=%.0f%%  timeout=%.0f%%  meanPathEff=%.2f  meanMinClear=%.3fm\n', ...
        FAN_DEG_VALUES(fi), nSucc, n, 100*nSucc/n, 100*lo, 100*hi, ...
        100*nCrash/n, 100*nTimeout/n, mean([results(sel).pathEfficiency],'omitnan'), mean([results(sel).minClearance]));
end
fprintf('=====================================================================\n');

plot_sweep_fan_deg_Deck6_env4(results, FAN_DEG_VALUES);
end
