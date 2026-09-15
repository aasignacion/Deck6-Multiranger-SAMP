function results = run_dropout_xy_trajectories_MR_vs_V9_baseline_env2()
%RUN_DROPOUT_XY_TRAJECTORIES_MR_VS_V9_BASELINE_ENV2 Same scenario/decks
% as run_dropout_xy_trajectories_ablation_env2.m (MR vs V9, using the
% original simulate_controller/simulate_controller9 +
% ctrl_wall_extend_ref_mixed_opt1_sym21(_vl53l8cx9) pipeline -- NOT the
% offset-aware Deck6 pipeline built later this session), but trimmed
% from 8 configs (MR/V9 x baseline/memory/reflex/both) x 6 dropout
% levels down to just:
%
%   MR-baseline  - Multiranger
%   V9-baseline  - vl53l8cx (9-beam)
%
% both at enable_reflex_safety=false, enable_memory_ttc=false, and only
% the 0% dropout level.
%
% Requires in path: load_sdf_obbs, collision_sphere_obbs,
% raycast_ranges_obbs, raycast_ranges9_obbs, apply_range_noise,
% simulate_controller, simulate_controller9,
% ctrl_wall_extend_ref_mixed_opt1_sym21(_vl53l8cx9),
% plot_dropout_xy_trajectories_ablation.

clc;

% ====================== SCENARIO (same as *_ablation_env2.m) ======================
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
simBase.exit_detect = struct();   % defaults: front_m=1.2, side_m=1.0, up_m=0.8, hold_s=0.6

% ====================== dropout sweep (0% only) ======================
dropoutLevels = 0.00;
levelLabels   = "0%";
levelColors   = {[0 0 0]};
N_PER_LEVEL   = 10;   % kept small for a fast first pass -- raise once this runs cleanly

% ====================== configs: baseline only, MR vs V9 ======================
MRCtrl = @ctrl_wall_extend_ref_mixed_opt1_sym21;
V9Ctrl = @ctrl_wall_extend_ref_mixed_opt1_sym21_vl53l8cx9;
configs = struct( ...
    'name',                 {"MR-baseline",         "V9-baseline"}, ...
    'simFcn',                {@simulate_controller,  @simulate_controller9}, ...
    'ctrlFcn',               {MRCtrl,                V9Ctrl}, ...
    'enable_reflex_safety', {false,                 false}, ...
    'enable_memory_ttc',    {false,                 false});

results = struct('level',{}, 'dropout_pct',{}, 'trial',{}, 'config',{}, 'x',{}, 'y',{}, ...
    'endX',{}, 'endY',{}, 'success',{}, 'collision',{}, 'color',{}, 'label',{});

for ci = 1:numel(configs)
    cfg = configs(ci);

    for li = 1:numel(dropoutLevels)
        dp = dropoutLevels(li);
        fprintf('=== %s -- Dropout %s ===\n', cfg.name, levelLabels(li));

        for trial = 1:N_PER_LEVEL
            rng(ci*10000 + li*1000 + trial, 'twister');

            sim = simBase;
            sim.sensor_noise = struct( ...
                'sigma0', 0.005, ...
                'sigma_slope', 0.01, ...
                'dropout_prob', dp, ...
                'max_range', env.max_range);
            sim.bug_params = struct( ...
                'debug_print', false, ...
                'enable_reflex_safety', cfg.enable_reflex_safety, ...
                'enable_memory_ttc', cfg.enable_memory_ttc);

            log = cfg.simFcn(cfg.ctrlFcn, sprintf('%s dp=%s trial=%d', cfg.name, levelLabels(li), trial), env, sim);

            idx = numel(results) + 1;
            results(idx).level = li;
            results(idx).dropout_pct = levelLabels(li);
            results(idx).trial = trial;
            results(idx).config = cfg.name;
            results(idx).x = log.x;
            results(idx).y = log.y;
            results(idx).endX = log.x(end);
            results(idx).endY = log.y(end);
            results(idx).success = log.goal_reached || log.exited;
            results(idx).collision = log.collision;
            results(idx).color = levelColors{li};
            results(idx).label = levelLabels(li);

            fprintf('  trial %2d: goal_reached=%d exited=%d collision=%d stop_reason=%s\n', ...
                trial, log.goal_reached, log.exited, log.collision, log.stop_reason);
        end
    end
end

% ====================== checkpoint (BEFORE plotting) ======================
matPath = fullfile(pwd, 'dropout_xy_trajectories_MR_vs_V9_baseline_env2_results.mat');
save(matPath, 'results', 'env', 'levelLabels', 'levelColors', 'S', '-v7.3');
fprintf('\nRaw results checkpointed to: %s\n', matPath);

% ====================== summary ======================
fprintf('\n=== Summary (Success/Crash/Timeout (%%) per config per level, n=%d) ===\n', N_PER_LEVEL);
configNames = [configs.name];
for ci = 1:numel(configNames)
    fprintf('--- %s ---\n', configNames(ci));
    for li = 1:numel(dropoutLevels)
        sel = ([results.level] == li) & ([results.config] == configNames(ci));
        n = sum(sel);
        nSucc = sum([results(sel).success]);
        nCrash = sum([results(sel).collision]);
        nTimeout = n - nSucc - nCrash;
        fprintf('  %4s dropout: %.0f/%.0f/%.0f %%\n', ...
            levelLabels(li), 100*nSucc/n, 100*nCrash/n, 100*nTimeout/n);
    end
end

plot_dropout_xy_trajectories_ablation(results, env.obbs, levelLabels, levelColors, configNames, ...
    'XLim', [-1 5], 'YLim', [-1 7]);

figPath = fullfile(pwd, 'dropout_xy_trajectories_MR_vs_V9_baseline_env2.png');
exportgraphics(gcf, figPath, 'Resolution', 200);
fprintf('\nFigure saved to: %s\n', figPath);
end
