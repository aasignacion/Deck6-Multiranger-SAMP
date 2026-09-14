function plot_sweep_dmin_corner_metrics_MR_vs_V9_env104(results, DMIN_VALUES)
%PLOT_SWEEP_DMIN_CORNER_METRICS_MR_VS_V9_ENV104 Corner-handling/
% mode-switching metrics (# corner episodes, mode-time fractions, #
% mode transitions) vs d_min, legend "SAMP-MR" / "SAMP-6R". Call with no
% arguments to load the saved checkpoint and replot without rerunning.

if nargin < 1
    S_ = load('sweep_dmin_MR_vs_V9_env104_results.mat', 'results', 'DMIN_VALUES');
    results = S_.results;
    DMIN_VALUES = S_.DMIN_VALUES;
end

condStrs = string({results.cond});
condNames = ["SAMP-MR", "SAMP-6R"];
colors = containers.Map({'SAMP-MR','SAMP-6R'}, {[0.10 0.40 0.85], [0.15 0.65 0.25]});

panels = {
    'nCornerEpisodes',  '# corner episodes'
    'nModeTransitions', '# mode transitions'
    'fracCorner',       'fraction time: corner mode'
    'fracWall',         'fraction time: wall mode'
    'fracCruise',       'fraction time: cruise mode'
    'pathEfficiency',   'path efficiency'
    };

fig = figure('Name', 'd_min sweep: corner-handling metrics', 'Color', 'w', ...
    'Position', [80 80 1200 700]);

for pIdx = 1:size(panels,1)
    fn = panels{pIdx,1};
    desc = panels{pIdx,2};
    subplot(2,3,pIdx);
    hold on; grid on; box on;
    for ci = 1:numel(condNames)
        n = numel(DMIN_VALUES);
        vals = nan(1,n);
        for di = 1:n
            sel = (condStrs == condNames(ci)) & ([results.d_min] == DMIN_VALUES(di));
            vals(di) = mean([results(sel).(fn)], 'omitnan');
        end
        c = colors(char(condNames(ci)));
        plot(DMIN_VALUES, vals, 'o-', 'Color', c, 'LineWidth', 1.8, ...
            'MarkerFaceColor', c, 'MarkerSize', 6, 'DisplayName', condNames(ci));
    end
    xlabel('d\_min (m)'); ylabel(desc);
    title(desc);
    if pIdx == 1
        legend('Location', 'best');
    end
end
sgtitle('Corner-handling / mode-switching metrics vs d\_min -- Env104, baseline, 0% dropout');

figPath = fullfile(pwd, 'sweep_dmin_corner_metrics_MR_vs_V9_env104.png');
exportgraphics(fig, figPath, 'Resolution', 200);
fprintf('Saved figure to %s\n', figPath);
end
