function results = run_sweep_dmin_MR_vs_V9_env104()
%RUN_SWEEP_DMIN_MR_VS_V9_ENV104 Sweeps the controller's safety margin
% (p.d_min) across 0.08/0.12/0.16 m, comparing MR ("SAMP-MR") vs V9
% ("SAMP-6R") on Env104, baseline config (no RSL, no SMM), 0% dropout,
% matching the scenario/speed of run_dropout_xy_trajectories_MR_vs_V9_
% baseline_env2.m (v_limits=[0.7 0.3 0.3]) where SAMP-MR got 0/10 and
% SAMP-6R got 10/10 success at the default d_min=0.12. This checks
% whether that gap is sensitive to d_min or holds across the range.
%
% Uses the original zero-offset MR/V9 pipeline (simulate_controller(9)_logmem
% + ctrl_wall_extend_ref_mixed_opt1_sym21(_vl53l8cx9)), matching the
% "SAMP-MR"/"SAMP-6R" naming convention used elsewhere this session for
% this specific MR-vs-V9 comparison.
%
% Also computes corner-handling/mode-switching metrics per trial via
% compute_nav_metrics.m (nCornerEpisodes, cornerDurMean, fracCorner/
% fracWall/fracCruise, nModeTransitions) -- the simulator already
% returns everything these need (mem.corner_mode/wall_mode/wall_conf/
% turn_dir logged verbatim), this was just never being extracted before.
%
% Requires in path: load_sdf_obbs, collision_sphere_obbs,
% raycast_ranges_obbs, raycast_ranges9_obbs, apply_range_noise,
% simulate_controller_logmem, simulate_controller9_logmem,
% ctrl_wall_extend_ref_mixed_opt1_sym21(_vl53l8cx9), compute_nav_metrics, wilson_ci.

clc;

S = struct();
S.sdf_path = 'Env104_combined_trans.sdf';
S.init = struct('x',0.0,'y',0.0,'z',0.25,'yaw',0.0);
S.goal = struct('x',4.0,'y',1.0,'z',0.25);
S.T = 1000;

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

DMIN_VALUES = [0.08, 0.12, 0.16];
N_TRIALS    = 3;
D_MIN_METRIC = 0.12;   % fixed threshold for the fracBelowDMin *metric*,
                       % held constant across the sweep so it stays
                       % comparable cell-to-cell even as the
                       % controller's own operating d_min changes --
                       % NOT the swept parameter itself.

configs = struct( ...
    'name',   {"SAMP-MR",             "SAMP-6R"}, ...
    'simFcn', {@simulate_controller_logmem,  @simulate_controller9_logmem}, ...
    'ctrlFcn',{@ctrl_wall_extend_ref_mixed_opt1_sym21, @ctrl_wall_extend_ref_mixed_opt1_sym21_vl53l8cx9});

results = struct('cond', {}, 'd_min', {}, 'trial', {}, 'stop_reason', {}, 'success', {}, 'collision', {}, ...
    'nCornerEpisodes', {}, 'cornerDurMean', {}, 'nTurnFlips', {}, ...
    'nModeTransitions', {}, 'fracCorner', {}, 'fracWall', {}, 'fracCruise', {}, ...
    'pathEfficiency', {}, 'totalTime', {}, 'meanSpeed', {}, 'minClearance', {});

for ci = 1:numel(configs)
    cfg = configs(ci);
    for di = 1:numel(DMIN_VALUES)
        dMinVal = DMIN_VALUES(di);
        fprintf('\n=== %s, d_min=%.2f (baseline, 0%% dropout, %d trials) ===\n', cfg.name, dMinVal, N_TRIALS);

        for trial = 1:N_TRIALS
            rng(ci*100000 + di*1000 + trial, 'twister');

            sim = simBase;
            sim.sensor_noise = struct('sigma0', 0.005, 'sigma_slope', 0.01, ...
                'dropout_prob', 0.0, 'max_range', env.max_range);
            sim.bug_params = struct('debug_print', false, ...
                'enable_reflex_safety', false, 'enable_memory_ttc', false, ...
                'd_min', dMinVal);

            log = cfg.simFcn(cfg.ctrlFcn, sprintf('%s d_min=%.2f trial=%d', cfg.name, dMinVal, trial), env, sim);
            m = compute_nav_metrics(log, D_MIN_METRIC);

            idx = numel(results) + 1;
            results(idx).cond = cfg.name;
            results(idx).d_min = dMinVal;
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

matPath = fullfile(pwd, 'sweep_dmin_MR_vs_V9_env104_results.mat');
save(matPath, 'results', 'S', 'DMIN_VALUES', 'N_TRIALS');
fprintf('\nRaw results checkpointed to: %s\n', matPath);

fprintf('\n=====================================================================\n');
fprintf('d_min SWEEP SUMMARY (n=%d trials each)\n', N_TRIALS);
fprintf('=====================================================================\n');
condStrs = string({results.cond});
condNames = unique(condStrs, 'stable');
for ci = 1:numel(condNames)
    for di = 1:numel(DMIN_VALUES)
        sel = (condStrs == condNames(ci)) & ([results.d_min] == DMIN_VALUES(di));
        n = sum(sel);
        nSucc = sum([results(sel).success]);
        [lo, hi] = wilson_ci(nSucc, n);
        fprintf('  %-8s d_min=%.2f: success=%d/%d (%.0f%%, 95%% CI [%.0f%%,%.0f%%])\n', ...
            condNames(ci), DMIN_VALUES(di), nSucc, n, 100*nSucc/n, 100*lo, 100*hi);
    end
end
fprintf('=====================================================================\n');

fprintf('\n=====================================================================\n');
fprintf('CORNER-HANDLING / MODE-SWITCHING METRICS vs d_min (mean over n=%d)\n', N_TRIALS);
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
        for di = 1:numel(DMIN_VALUES)
            sel = (condStrs == condNames(ci)) & ([results.d_min] == DMIN_VALUES(di));
            vals = [results(sel).(fn)];
            fprintf('%s@%.2f=%.3f  ', condNames(ci), DMIN_VALUES(di), mean(vals, 'omitnan'));
        end
    end
    fprintf('\n');
end
fprintf('=====================================================================\n');

plot_sweep_dmin_MR_vs_V9_env104(results, DMIN_VALUES);
plot_sweep_dmin_corner_metrics_MR_vs_V9_env104(results, DMIN_VALUES);
end
