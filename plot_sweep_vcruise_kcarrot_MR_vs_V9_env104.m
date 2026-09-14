function plot_sweep_vcruise_kcarrot_MR_vs_V9_env104(results, VCRUISE_VALUES, KCARROT_VALUES)
%PLOT_SWEEP_VCRUISE_KCARROT_MR_VS_V9_ENV104 Heatmaps of success rate and
% mean corner-episode count over the v_cruise x k_carrot grid, one
% column per condition (SAMP-MR / SAMP-6R). Call with no arguments to
% load the saved checkpoint and replot without rerunning.

if nargin < 1
    S_ = load('sweep_vcruise_kcarrot_MR_vs_V9_env104_results.mat', 'results', 'VCRUISE_VALUES', 'KCARROT_VALUES');
    results = S_.results;
    VCRUISE_VALUES = S_.VCRUISE_VALUES;
    KCARROT_VALUES = S_.KCARROT_VALUES;
end

condStrs = string({results.cond});
condNames = ["SAMP-MR", "SAMP-6R"];
nV = numel(VCRUISE_VALUES); nK = numel(KCARROT_VALUES);

fig = figure('Name', 'v_cruise x k_carrot sweep: SAMP-MR vs SAMP-6R (Env104)', 'Color', 'w', ...
    'Position', [60 60 1000 750]);

for ci = 1:numel(condNames)
    successGrid = nan(nV, nK);
    cornerGrid = nan(nV, nK);
    for vi = 1:nV
        for ki = 1:nK
            sel = (condStrs == condNames(ci)) & ([results.v_cruise] == VCRUISE_VALUES(vi)) & ...
                ([results.k_carrot] == KCARROT_VALUES(ki));
            successGrid(vi,ki) = 100 * mean([results(sel).success]);
            cornerGrid(vi,ki) = mean([results(sel).nCornerEpisodes]);
        end
    end

    subplot(2, 2, ci);
    imagesc(KCARROT_VALUES, VCRUISE_VALUES, successGrid); axis xy;
    colormap(gca, parula); cb = colorbar; ylabel(cb, 'success rate (%)');
    caxis([0 100]);
    xlabel('k\_carrot (k\_v)'); ylabel('v\_cruise (v\_max)');
    title(sprintf('%s: success rate', condNames(ci)));
    xticks(KCARROT_VALUES); yticks(VCRUISE_VALUES);
    for vi = 1:nV
        for ki = 1:nK
            text(KCARROT_VALUES(ki), VCRUISE_VALUES(vi), sprintf('%.0f', successGrid(vi,ki)), ...
                'HorizontalAlignment', 'center', 'Color', 'k', 'FontSize', 9);
        end
    end

    subplot(2, 2, ci+2);
    imagesc(KCARROT_VALUES, VCRUISE_VALUES, cornerGrid); axis xy;
    colormap(gca, parula); cb2 = colorbar; ylabel(cb2, 'mean # corner episodes');
    xlabel('k\_carrot (k\_v)'); ylabel('v\_cruise (v\_max)');
    title(sprintf('%s: corner episodes', condNames(ci)));
    xticks(KCARROT_VALUES); yticks(VCRUISE_VALUES);
    for vi = 1:nV
        for ki = 1:nK
            text(KCARROT_VALUES(ki), VCRUISE_VALUES(vi), sprintf('%.1f', cornerGrid(vi,ki)), ...
                'HorizontalAlignment', 'center', 'Color', 'k', 'FontSize', 9);
        end
    end
end

sgtitle('v\_cruise (v\_max) x k\_carrot (k\_v) sweep -- Env104, baseline, 0% dropout');

figPath = fullfile(pwd, 'sweep_vcruise_kcarrot_MR_vs_V9_env104.png');
exportgraphics(fig, figPath, 'Resolution', 200);
fprintf('Saved figure to %s\n', figPath);
end
