function results = run_sweep_vcruise_kcarrot_MR_vs_V9_env104()
%RUN_SWEEP_VCRUISE_KCARROT_MR_VS_V9_ENV104 2D sweep over the
% straight-tunnel/wall-following speed cap (p.v_cruise, "v_max") and its
% proportional gain (p.k_carrot, "k_v" -- vmag = k_carrot*x_carrot,
% capped at v_cruise, in ctrl_wall_extend_ref_mixed_opt1_sym21(_vl53l8cx9).m's
% wall_mode branch), comparing MR ("SAMP-MR") vs V9 ("SAMP-6R") on
% Env104, baseline config (no RSL, no SMM), 0% dropout.
%
% Uses the original zero-offset MR/V9 pipeline (simulate_controller(9)_logmem
% + ctrl_wall_extend_ref_mixed_opt1_sym21(_vl53l8cx9)), matching the
% "SAMP-MR"/"SAMP-6R" naming convention used elsewhere this session, and
% includes corner-handling/mode-switching metrics via compute_nav_metrics.m
% from the start (earlier single-parameter sweeps needed a follow-up
% pass to add these).
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

VCRUISE_VALUES = [0.10, 0.16, 0.22];   % "v_max" -- straight-tunnel/wall-following speed cap
KCARROT_VALUES = [0.75, 1.5, 3.0];     % "k_v"   -- gain from front clearance to commanded speed
N_TRIALS = 3;
D_MIN_METRIC = 0.12;

configs = struct( ...
    'name',   {"SAMP-MR",             "SAMP-6R"}, ...
    'simFcn', {@simulate_controller_logmem,  @simulate_controller9_logmem}, ...
    'ctrlFcn',{@ctrl_wall_extend_ref_mixed_opt1_sym21, @ctrl_wall_extend_ref_mixed_opt1_sym21_vl53l8cx9});

results = struct('cond', {}, 'v_cruise', {}, 'k_carrot', {}, 'trial', {}, 'stop_reason', {}, ...
    'success', {}, 'collision', {}, 'nCornerEpisodes', {}, 'nModeTransitions', {}, ...
    'fracCorner', {}, 'fracWall', {}, 'fracCruise', {}, 'pathEfficiency', {}, ...
    'totalTime', {}, 'meanSpeed', {}, 'minClearance', {});

for ci = 1:numel(configs)
    cfg = configs(ci);
    for vi = 1:numel(VCRUISE_VALUES)
        for ki = 1:numel(KCARROT_VALUES)
            vVal = VCRUISE_VALUES(vi);
            kVal = KCARROT_VALUES(ki);
            fprintf('\n=== %s, v_cruise=%.2f, k_carrot=%.2f (baseline, 0%% dropout, %d trials) ===\n', ...
                cfg.name, vVal, kVal, N_TRIALS);

            for trial = 1:N_TRIALS
                rng(ci*1000000 + vi*10000 + ki*1000 + trial, 'twister');

                sim = simBase;
                sim.sensor_noise = struct('sigma0', 0.005, 'sigma_slope', 0.01, ...
                    'dropout_prob', 0.0, 'max_range', env.max_range);
                sim.bug_params = struct('debug_print', false, ...
                    'enable_reflex_safety', false, 'enable_memory_ttc', false, ...
                    'v_cruise', vVal, 'k_carrot', kVal);

                log = cfg.simFcn(cfg.ctrlFcn, sprintf('%s v_cruise=%.2f k_carrot=%.2f trial=%d', ...
                    cfg.name, vVal, kVal, trial), env, sim);
                m = compute_nav_metrics(log, D_MIN_METRIC);

                idx = numel(results) + 1;
                results(idx).cond = cfg.name;
                results(idx).v_cruise = vVal;
                results(idx).k_carrot = kVal;
                results(idx).trial = trial;
                results(idx).stop_reason = char(log.stop_reason);
                results(idx).success = log.goal_reached || log.exited;
                results(idx).collision = log.collision;
                results(idx).nCornerEpisodes = m.nCornerEpisodes;
                results(idx).nModeTransitions = m.nModeTransitions;
                results(idx).fracCorner = m.fracCorner;
                results(idx).fracWall = m.fracWall;
                results(idx).fracCruise = m.fracCruise;
                results(idx).pathEfficiency = m.pathEfficiency;
                results(idx).totalTime = m.totalTime;
                results(idx).meanSpeed = m.meanSpeed;
                results(idx).minClearance = m.minClearance;

                fprintf('  trial %2d: stop=%-16s success=%d corners=%2d fracWall=%.2f pathEff=%.2f\n', ...
                    trial, results(idx).stop_reason, results(idx).success, m.nCornerEpisodes, m.fracWall, m.pathEfficiency);
            end
        end
    end
end

matPath = fullfile(pwd, 'sweep_vcruise_kcarrot_MR_vs_V9_env104_results.mat');
save(matPath, 'results', 'S', 'VCRUISE_VALUES', 'KCARROT_VALUES', 'N_TRIALS', 'D_MIN_METRIC');
fprintf('\nRaw results checkpointed to: %s\n', matPath);

fprintf('\n=====================================================================\n');
fprintf('SUCCESS RATE GRIDS (rows=v_cruise, cols=k_carrot), n=%d trials/cell\n', N_TRIALS);
fprintf('=====================================================================\n');
condStrs = string({results.cond});
condNames = unique(condStrs, 'stable');
for ci = 1:numel(condNames)
    fprintf('\n%s:\n', condNames(ci));
    fprintf('%12s', 'v_cru\\k_car');
    for ki = 1:numel(KCARROT_VALUES)
        fprintf('%10.2f', KCARROT_VALUES(ki));
    end
    fprintf('\n');
    for vi = 1:numel(VCRUISE_VALUES)
        fprintf('%12.2f', VCRUISE_VALUES(vi));
        for ki = 1:numel(KCARROT_VALUES)
            sel = (condStrs == condNames(ci)) & ([results.v_cruise] == VCRUISE_VALUES(vi)) & ...
                ([results.k_carrot] == KCARROT_VALUES(ki));
            n = sum(sel);
            nSucc = sum([results(sel).success]);
            fprintf('%9.0f%%', 100*nSucc/n);
        end
        fprintf('\n');
    end
end

fprintf('\n=====================================================================\n');
fprintf('CORNER-HANDLING METRICS (mean nCornerEpisodes), rows=v_cruise, cols=k_carrot\n');
fprintf('=====================================================================\n');
for ci = 1:numel(condNames)
    fprintf('\n%s:\n', condNames(ci));
    fprintf('%12s', 'v_cru\\k_car');
    for ki = 1:numel(KCARROT_VALUES)
        fprintf('%10.2f', KCARROT_VALUES(ki));
    end
    fprintf('\n');
    for vi = 1:numel(VCRUISE_VALUES)
        fprintf('%12.2f', VCRUISE_VALUES(vi));
        for ki = 1:numel(KCARROT_VALUES)
            sel = (condStrs == condNames(ci)) & ([results.v_cruise] == VCRUISE_VALUES(vi)) & ...
                ([results.k_carrot] == KCARROT_VALUES(ki));
            fprintf('%10.2f', mean([results(sel).nCornerEpisodes]));
        end
        fprintf('\n');
    end
end
fprintf('=====================================================================\n');

plot_sweep_vcruise_kcarrot_MR_vs_V9_env104(results, VCRUISE_VALUES, KCARROT_VALUES);
end
