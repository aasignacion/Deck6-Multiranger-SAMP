function results = run_sweep_speed_MR_vs_V9_env104()
%RUN_SWEEP_SPEED_MR_VS_V9_ENV104 Sweeps the controller's cruise speed
% (p.v_cruise) across 0.10/0.16/0.22 m/s, comparing MR ("SAMP-MR") vs V9
% ("SAMP-6R") on Env104, baseline config (no RSL, no SMM), 0% dropout,
% same scenario as run_sweep_dmin_MR_vs_V9_env104.m (v_limits=
% [0.7 0.3 0.3], the sim-level speed CAP -- p.v_cruise is the
% controller's own internal target speed, always <= that cap).
%
% Uses the original zero-offset MR/V9 pipeline, matching the "SAMP-MR"/
% "SAMP-6R" naming convention used elsewhere this session.
%
% Also computes corner-handling/mode-switching metrics per trial via
% compute_nav_metrics.m (nCornerEpisodes, cornerDurMean, fracCorner/
% fracWall/fracCruise, nModeTransitions).
%
% Requires in path: load_sdf_obbs, collision_sphere_obbs,
% raycast_ranges_obbs, raycast_ranges9_obbs, apply_range_noise,
% simulate_controller_logmem, simulate_controller9_logmem,
% ctrl_wall_extend_ref_mixed_opt1_sym21(_vl53l8cx9), compute_nav_metrics, wilson_ci.

clc;

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
simBase.v_limits = [0.7 0.3 0.3];
simBase.w_limit  = 1.5;
simBase.z_limits = [0.05 5.0];
simBase.init = S.init;
simBase.goal = S.goal;
simBase.is_vertical = false;
simBase.exit_detect = struct();

VCRUISE_VALUES = [0.10, 0.16, 0.22];
N_TRIALS       = 10;
D_MIN_METRIC   = 0.12;   % fixed threshold for the fracBelowDMin *metric*, not the swept parameter

configs = struct( ...
    'name',   {"SAMP-MR",             "SAMP-6R"}, ...
    'simFcn', {@simulate_controller_logmem,  @simulate_controller9_logmem}, ...
    'ctrlFcn',{@ctrl_wall_extend_ref_mixed_opt1_sym21, @ctrl_wall_extend_ref_mixed_opt1_sym21_vl53l8cx9});

results = struct('cond', {}, 'v_cruise', {}, 'trial', {}, 'stop_reason', {}, 'success', {}, 'collision', {}, ...
    'nCornerEpisodes', {}, 'cornerDurMean', {}, 'nTurnFlips', {}, ...
    'nModeTransitions', {}, 'fracCorner', {}, 'fracWall', {}, 'fracCruise', {}, ...
    'pathEfficiency', {}, 'totalTime', {}, 'meanSpeed', {}, 'minClearance', {});

for ci = 1:numel(configs)
    cfg = configs(ci);
    for vi = 1:numel(VCRUISE_VALUES)
        vVal = VCRUISE_VALUES(vi);
        fprintf('\n=== %s, v_cruise=%.2f (baseline, 0%% dropout, %d trials) ===\n', cfg.name, vVal, N_TRIALS);

        for trial = 1:N_TRIALS
            rng(ci*100000 + vi*1000 + trial, 'twister');

            sim = simBase;
            sim.sensor_noise = struct('sigma0', 0.005, 'sigma_slope', 0.01, ...
                'dropout_prob', 0.0, 'max_range', env.max_range);
            sim.bug_params = struct('debug_print', false, ...
                'enable_reflex_safety', false, 'enable_memory_ttc', false, ...
                'v_cruise', vVal);

            log = cfg.simFcn(cfg.ctrlFcn, sprintf('%s v_cruise=%.2f trial=%d', cfg.name, vVal, trial), env, sim);
            m = compute_nav_metrics(log, D_MIN_METRIC);

            idx = numel(results) + 1;
            results(idx).cond = cfg.name;
            results(idx).v_cruise = vVal;
            results(idx).trial = trial;
            results(idx).stop_reason = char(log.stop_reason);
            results(idx).success = log.goal_reached || log.exited;
            results(idx).collision = log.collision;
            results(idx).nCornerEpisodes = m.nCornerEpisodes;
            results(idx).cornerDurMean = m.cornerDurMean;
            results(idx).nTurnFlips = m.nTurnFlips;
            results(idx).nModeTransitions = m.nModeTransitions;
            results(idx).fracCorner = m.fracCorner;
            results(idx).fracWall = m.fracWall;
            results(idx).fracCruise = m.fracCruise;
            results(idx).pathEfficiency = m.pathEfficiency;
            results(idx).totalTime = m.totalTime;
            results(idx).meanSpeed = m.meanSpeed;
            results(idx).minClearance = m.minClearance;

            fprintf('  trial %2d: stop=%-16s success=%d corners=%2d fracCorner=%.2f fracWall=%.2f fracCruise=%.2f\n', ...
                trial, results(idx).stop_reason, results(idx).success, m.nCornerEpisodes, m.fracCorner, m.fracWall, m.fracCruise);
        end
    end
end

matPath = fullfile(pwd, 'sweep_speed_MR_vs_V9_env104_results.mat');
save(matPath, 'results', 'S', 'VCRUISE_VALUES', 'N_TRIALS');
fprintf('\nRaw results checkpointed to: %s\n', matPath);

fprintf('\n=====================================================================\n');
fprintf('v_cruise SWEEP SUMMARY (n=%d trials each)\n', N_TRIALS);
fprintf('=====================================================================\n');
condStrs = string({results.cond});
condNames = unique(condStrs, 'stable');
for ci = 1:numel(condNames)
    for vi = 1:numel(VCRUISE_VALUES)
        sel = (condStrs == condNames(ci)) & ([results.v_cruise] == VCRUISE_VALUES(vi));
        n = sum(sel);
        nSucc = sum([results(sel).success]);
        [lo, hi] = wilson_ci(nSucc, n);
        fprintf('  %-8s v_cruise=%.2f: success=%d/%d (%.0f%%, 95%% CI [%.0f%%,%.0f%%])\n', ...
            condNames(ci), VCRUISE_VALUES(vi), nSucc, n, 100*nSucc/n, 100*lo, 100*hi);
    end
end
fprintf('=====================================================================\n');

fprintf('\n=====================================================================\n');
fprintf('CORNER-HANDLING / MODE-SWITCHING METRICS vs v_cruise (mean over n=%d)\n', N_TRIALS);
fprintf('=====================================================================\n');
fields_to_report = {
    'nCornerEpisodes',  '# corner episodes'
    'cornerDurMean',    'mean corner episode duration (s)'
    'nModeTransitions', '# mode transitions'
    'fracCorner',       'fraction of time in corner mode'
    'fracWall',         'fraction of time in wall mode'
    'fracCruise',       'fraction of time in cruise mode'
    'pathEfficiency',   'path efficiency'
    'minClearance',     'minimum clearance (m)'
    };
for f = 1:size(fields_to_report,1)
    fn = fields_to_report{f,1};
    desc = fields_to_report{f,2};
    fprintf('%-38s ', desc);
    for ci = 1:numel(condNames)
        for vi = 1:numel(VCRUISE_VALUES)
            sel = (condStrs == condNames(ci)) & ([results.v_cruise] == VCRUISE_VALUES(vi));
            vals = [results(sel).(fn)];
            fprintf('%s@%.2f=%.3f  ', condNames(ci), VCRUISE_VALUES(vi), mean(vals, 'omitnan'));
        end
    end
    fprintf('\n');
end
fprintf('=====================================================================\n');

plot_sweep_speed_MR_vs_V9_env104(results, VCRUISE_VALUES);
plot_sweep_speed_corner_metrics_MR_vs_V9_env104(results, VCRUISE_VALUES);
end
