function plot_sweep_fan_deg_Deck6_env4(results, FAN_DEG_VALUES)
%PLOT_SWEEP_FAN_DEG_DECK6_ENV4 Plots success rate (with Wilson 95% CI),
% mean path efficiency, and mean minimum clearance vs. the r_FL/r_FR
% mounting fan angle, from run_sweep_fan_deg_Deck6_env4.m's results.
%
% Call with no arguments to load the saved checkpoint
% (sweep_fan_deg_Deck6_env4_results.mat) and plot that instead of
% re-running the sweep -- e.g. just to regenerate the figure.

if nargin < 1
    S_ = load('sweep_fan_deg_Deck6_env4_results.mat', 'results', 'FAN_DEG_VALUES');
    results = S_.results;
    FAN_DEG_VALUES = S_.FAN_DEG_VALUES;
end

fanVals = [results.fan_deg];
nF = numel(FAN_DEG_VALUES);

successRate = nan(1,nF); ciLo = nan(1,nF); ciHi = nan(1,nF);
meanPathEff = nan(1,nF); meanMinClear = nan(1,nF); n = zeros(1,nF);

for fi = 1:nF
    sel = (fanVals == FAN_DEG_VALUES(fi));
    n(fi) = sum(sel);
    nSucc = sum([results(sel).success]);
    successRate(fi) = nSucc / n(fi);
    tolTime(fi) = results(sel).totalTime;
    [lo, hi] = wilson_ci(nSucc, n(fi));
    ciLo(fi) = lo; ciHi(fi) = hi;
    meanPathEff(fi) = mean([results(sel).pathEfficiency], 'omitnan');
    meanMinClear(fi) = mean([results(sel).minClearance], 'omitnan');
end

fig = figure('Name', 'Deck6 FAN_DEG sweep (Env4)', 'Color', 'w', ...
    'Visible', 'on', 'Position', [100 100 1300 380]);

subplot(1,3,1);
% hold on; grid on; box on;
% errLo = 100*(successRate - ciLo);
% errHi = 100*(ciHi - successRate);
% errorbar(FAN_DEG_VALUES, 100*successRate, errLo, errHi, 'o-', ...
%     'LineWidth', 1.5, 'MarkerFaceColor', 'auto', 'CapSize', 8);
% xline(22.5, '--', 'real hardware', 'Color', [0.5 0.5 0.5], 'LabelVerticalAlignment', 'bottom');
plot(FAN_DEG_VALUES, 100*successRate, 'o-', 'LineWidth', 1.5);
ylim([-5 105]);
xlabel('FAN\_DEG (deg)'); ylabel('Success rate (%)');
title(sprintf('Success rate (95%% CI, n=%d/pt)', n(1)));
grid on;

subplot(1,3,2);
hold on; grid on; box on;
plot(FAN_DEG_VALUES, tolTime, 'o-', 'LineWidth', 1.5);
xline(22.5, '--', 'Color', [0.5 0.5 0.5]);
xlabel('FAN\_DEG (deg)'); ylabel('Total Time');
title('Path efficiency (straight-line / path length)');

subplot(1,3,3);
hold on; grid on; box on;
plot(FAN_DEG_VALUES, meanMinClear, 'o-', 'LineWidth', 1.5);
xline(22.5, '--', 'Color', [0.5 0.5 0.5]);
xlabel('FAN\_DEG (deg)'); ylabel('Mean min clearance (m)');
title('Minimum clearance');

sgtitle('Deck6 baseline on Env4: sensitivity to r_{FL}/r_{FR} mounting angle', 'Interpreter', 'tex');

outPng = fullfile(pwd, 'sweep_fan_deg_Deck6_env4.png');
exportgraphics(fig, outPng, 'Resolution', 200);
fprintf('Saved figure to %s\n', outPng);
end
