function plot_dropout_xy_trajectories_ablation(results, obbs, levelLabels, levelColors, configNames, varargin)
%PLOT_DROPOUT_XY_TRAJECTORIES_ABLATION 2x2 subplot, one panel per
% ablation configuration (see run_dropout_xy_trajectories_ablation.m),
% each panel styled like plot_dropout_xy_trajectories.m: thin
% semi-transparent trajectory lines colored by dropout level, small
% unfilled circle = success/exited, x = crash, square = other (timeout).
%
% INPUTS
%   results      - struct array with .config (one of configNames), plus
%                  the usual .x, .y, .endX, .endY, .success, .collision,
%                  .color, .label (see run_dropout_xy_trajectories_ablation.m)
%   obbs         - OBB struct array from load_sdf_obbs, or [] to skip walls.
%   levelLabels  - string array, one per dropout level, for the legend
%   levelColors  - cell array of 1x3 RGB, same order as levelLabels
%   configNames  - string array of the 4 (or fewer) config names, in the
%                  order panels should be laid out (row-major, 2x2 grid
%                  for 4 configs)
%
% NAME-VALUE OPTIONS
%   'XLim', 'YLim' - default [] (auto). E.g. 'XLim',[-1 5],'YLim',[-1 7].

p = inputParser;
addParameter(p, 'XLim', []);
addParameter(p, 'YLim', []);
parse(p, varargin{:});
opt = p.Results;

nCfg = numel(configNames);
nRows = ceil(sqrt(nCfg));
nCols = ceil(nCfg / nRows);

figure('Name', 'SAMP ablation: reflex safety layer vs memory/TTC', 'Color', 'w', ...
    'Position', [50 50 1400 900]);

for ci = 1:nCfg
    subplot(nRows, nCols, ci);
    hold on; grid on; box on; axis equal;

    if ~isempty(obbs)
        for i = 1:numel(obbs)
            o = obbs(i);
            hx = o.h(1); hy = o.h(2);
            localCorners = [ hx  hy 0; hx -hy 0; -hx -hy 0; -hx hy 0; hx hy 0]';
            worldCorners = o.R * localCorners + o.c;
            plot(worldCorners(1,:), worldCorners(2,:), '-', ...
                'Color', [0.75 0.75 0.75], 'LineWidth', 1, 'HandleVisibility', 'off');
        end
    end

    sel = [results.config] == configNames(ci);
    r = results(sel);

    for i = 1:numel(r)
        hLine = plot(r(i).x, r(i).y, '-', 'Color', r(i).color, 'LineWidth', 0.4, 'HandleVisibility', 'off');
        hLine.Color(4) = 0.35;
    end
    for i = 1:numel(r)
        if r(i).success
            marker = 'o';
        elseif r(i).collision
            marker = 'x';
        else
            marker = 's';
        end
        plot(r(i).endX, r(i).endY, marker, 'Color', r(i).color, 'MarkerFaceColor', 'none', ...
            'MarkerSize', 7, 'LineWidth', 1.6, 'HandleVisibility', 'off');
    end

    if ~isempty(opt.XLim); xlim(opt.XLim); end
    if ~isempty(opt.YLim); ylim(opt.YLim); end
    xlabel('x (m)'); ylabel('y (m)');
    title(configNames(ci), 'Interpreter', 'none');
end

% ---- shared legend (dropout-level colors), placed via an invisible axes ----
lgAx = axes('Position', [0 0 1 1], 'Visible', 'off');
hold(lgAx, 'on');
h = gobjects(numel(levelLabels),1);
for li = 1:numel(levelLabels)
    h(li) = plot(lgAx, nan, nan, '-', 'Color', levelColors{li}, 'LineWidth', 1.5, ...
        'DisplayName', levelLabels(li) + " dropout");
end
legend(lgAx, h, 'Location', 'southoutside', 'Orientation', 'horizontal');

sgtitle('SAMP ablation: reflex safety layer vs. memory/TTC (o = exited, x = crash)', 'Interpreter', 'none');
end
