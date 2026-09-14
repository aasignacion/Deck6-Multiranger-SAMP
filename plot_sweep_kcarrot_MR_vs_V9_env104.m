function plot_sweep_kcarrot_MR_vs_V9_env104(results, KCARROT_VALUES)
%PLOT_SWEEP_KCARROT_MR_VS_V9_ENV104 Success rate (Wilson 95% CI) and
% corner-handling/mode-switching metrics vs k_carrot, legend "SAMP-MR" /
% "SAMP-6R". Call with no arguments to load the saved checkpoint and
% replot without rerunning.

if nargin < 1
    S_ = load('sweep_kcarrot_MR_vs_V9_env104_results.mat', 'results', 'KCARROT_VALUES');
    results = S_.results;
    KCARROT_VALUES = S_.KCARROT_VALUES;
end

condStrs = string({results.cond});
condNames = ["SAMP-MR", "SAMP-6R"];
colors = containers.Map({'SAMP-MR','SAMP-6R'}, {[0.10 0.40 0.85], [0.15 0.65 0.25]});

fig = figure('Name', 'k_carrot sweep: SAMP-MR vs SAMP-6R (Env104)', 'Color', 'w', ...
    'Position', [60 60 1200 700]);

% -------- panel 1: success rate --------
subplot(2,3,1);
hold on; grid on; box on;
for ci = 1:numel(condNames)
    n = numel(KCARROT_VALUES);
    succRate = nan(1,n); ciLo = nan(1,n); ciHi = nan(1,n);
    for ki = 1:n
        sel = (condStrs == condNames(ci)) & ([results.k_carrot] == KCARROT_VALUES(ki));
        nSucc = sum([results(sel).success]);
        nTot = sum(sel);
        succRate(ki) = nSucc / nTot;
        [lo, hi] = wilson_ci(nSucc, nTot);
        ciLo(ki) = lo; ciHi(ki) = hi;
    end
    c = colors(char(condNames(ci)));
    errorbar(KCARROT_VALUES, 100*succRate, 100*(succRate-ciLo), 100*(ciHi-succRate), ...
        'o-', 'Color', c, 'LineWidth', 1.8, 'MarkerFaceColor', c, 'MarkerSize', 7, ...
        'CapSize', 8, 'DisplayName', condNames(ci));
end
xlabel('k\_carrot (k\_v)'); ylabel('Success rate (%)'); ylim([-5 105]);
title('Success rate'); legend('Location', 'best');

panels = {
    'nCornerEpisodes',  '# corner episodes'
    'nModeTransitions', '# mode transitions'
    'fracWall',         'fraction time: wall mode'
    'fracCruise',       'fraction time: cruise mode'
    'pathEfficiency',   'path efficiency'
    };

for pIdx = 1:size(panels,1)
    fn = panels{pIdx,1};
    desc = panels{pIdx,2};
    subplot(2,3,pIdx+1);
    hold on; grid on; box on;
    for ci = 1:numel(condNames)
        n = numel(KCARROT_VALUES);
        vals = nan(1,n);
        for ki = 1:n
            sel = (condStrs == condNames(ci)) & ([results.k_carrot] == KCARROT_VALUES(ki));
            vals(ki) = mean([results(sel).(fn)], 'omitnan');
        end
        c = colors(char(condNames(ci)));
        plot(KCARROT_VALUES, vals, 'o-', 'Color', c, 'LineWidth', 1.8, ...
            'MarkerFaceColor', c, 'MarkerSize', 6, 'DisplayName', condNames(ci));
    end
    xlabel('k\_carrot (k\_v)'); ylabel(desc); title(desc);
end

sgtitle('k\_carrot (k\_v) sweep -- Env104, baseline, 0% dropout, v\_cruise held at default (0.16 m/s)');

figPath = fullfile(pwd, 'sweep_kcarrot_MR_vs_V9_env104.png');
exportgraphics(fig, figPath, 'Resolution', 200);
fprintf('Saved figure to %s\n', figPath);
end
