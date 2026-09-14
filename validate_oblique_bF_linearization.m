function validate_oblique_bF_linearization()
%VALIDATE_OBLIQUE_BF_LINEARIZATION Validates the first-order
% approximation of the frontal asymmetry b_F = r_FL - r_FR for a pair of
% oblique front-fan rays against the exact ray-wall intersection
% formula, in a straight tunnel of width W, over a grid of lateral
% displacement e and heading error psi.
%
% Exact (ray-wall intersection, both rays hit their own side wall at
% y=+-W/2, no occlusion/truncation):
%   r_FL = (W/2 - e - a*sin(psi) - b*cos(psi)) / sin(theta+psi)
%   r_FR = (W/2 + e + a*sin(psi) - b*cos(psi)) / sin(theta-psi)
%
% Linearized (first order in e, psi), with h = W/2-b, L_theta = a + h*cot(theta):
%   r_FL ~= h/sin(theta) - (e + L_theta*psi)/sin(theta)
%   r_FR ~= h/sin(theta) + (e + L_theta*psi)/sin(theta)
%   b_F  ~= -(2/sin(theta)) * (e + L_theta*psi)
%
% Lateral comparison (ideal rays at +-pi/2, symmetric origins, a=b=0,
% theta=pi/2 -- a special case of the SAME exact formula, so this script
% reuses one function for both, as a consistency check):
%   r_L - r_R = -2e/cos(psi) ~= -2e
%
% Example geometry: the paper's 6-beam deck, r_FL/r_FR mounted at 5.5cm
% radius, +-22.5deg from the r_F axis (this session's established ICRA
% sensor geometry) -- a = 5.5cm*cos(22.5deg), b = 5.5cm*sin(22.5deg),
% theta = 22.5deg. Corridor width W = 0.55m, matching the physical
% tunnel modules used elsewhere this session (build_env1_tunnel.m /
% build_env3_tunnel.m).

clc;

% ============================== geometry ==============================
W = 0.55;                          % m, corridor width
mountR = 0.055;                    % m, sensor mount radius from vehicle center
fanDeg = 22.5;                     % deg, mounting/ray angle from the r_F axis
theta = deg2rad(fanDeg);
a = mountR * cos(theta);           % m, longitudinal offset
b = mountR * sin(theta);           % m, lateral offset

h = W/2 - b;
L_theta = a + h * cot_(theta);

fprintf('=====================================================================\n');
fprintf('GEOMETRY\n');
fprintf('=====================================================================\n');
fprintf('W (corridor width)      = %.4f m\n', W);
fprintf('mount radius            = %.4f m\n', mountR);
fprintf('theta (fan/ray angle)   = %.2f deg\n', fanDeg);
fprintf('a (longitudinal offset) = %.4f m\n', a);
fprintf('b (lateral offset)      = %.4f m\n', b);
fprintf('h = W/2 - b             = %.4f m\n', h);
fprintf('L_theta = a + h*cot(theta) = %.4f m   <- effective preview distance\n', L_theta);
fprintf('nominal r_FL=r_FR (e=psi=0), exact  = %.4f m\n', (W/2-b)/sin(theta));
fprintf('nominal r_FL=r_FR (e=psi=0), linear = %.4f m (= h/sin(theta))\n', h/sin(theta));

% ============================== 1D validation ==============================
% Sweep e alone (psi=0), and psi alone (e=0), comparing exact vs linear b_F.
e_range = linspace(-0.10, 0.10, 201);      % m -- up to ~36% of the half-width
psi_range = linspace(deg2rad(-15), deg2rad(15), 201);   % rad, safely below theta to avoid sin(theta-psi)->0

bF_exact_vs_e = arrayfun(@(e) bF_exact(e, 0, a, b, theta, W), e_range);
bF_lin_vs_e   = arrayfun(@(e) bF_linear(e, 0, theta, L_theta), e_range);

bF_exact_vs_psi = arrayfun(@(p) bF_exact(0, p, a, b, theta, W), psi_range);
bF_lin_vs_psi   = arrayfun(@(p) bF_linear(0, p, theta, L_theta), psi_range);

% ============================== 2D validation ==============================
[E, PSI] = meshgrid(e_range, psi_range);
BF_exact = arrayfun(@(e,p) bF_exact(e, p, a, b, theta, W), E, PSI);
BF_lin   = arrayfun(@(e,p) bF_linear(e, p, theta, L_theta), E, PSI);
absErr = abs(BF_exact - BF_lin);
relErr = absErr ./ max(abs(BF_exact), 1e-6) * 100;   % %, guarded near b_F=0

fprintf('\n=====================================================================\n');
fprintf('2D SWEEP SUMMARY (e in [%.2f,%.2f] m, psi in [%.1f,%.1f] deg)\n', ...
    e_range(1), e_range(end), rad2deg(psi_range(1)), rad2deg(psi_range(end)));
fprintf('=====================================================================\n');
fprintf('max |b_F_exact - b_F_linear|      = %.4f m\n', max(absErr(:)));
fprintf('mean |b_F_exact - b_F_linear|     = %.4f m\n', mean(absErr(:)));
fracUnder5pct = mean(relErr(:) < 5, 'omitnan');
fprintf('fraction of grid with rel. error < 5%%  = %.1f%%\n', 100*fracUnder5pct);

% ============================== lateral comparison ==============================
% Reuse the SAME exact oblique formula at theta=pi/2, a=0, b=0 -- this
% should reduce EXACTLY (not approximately) to r_L-r_R = -2e/cos(psi),
% confirming the general formula's consistency, not just its linearization.
e_test = 0.05; psi_test = deg2rad(10);
rL_minus_rR_viaGeneral = bF_exact(e_test, psi_test, 0, 0, pi/2, W);
rL_minus_rR_closedForm = -2*e_test/cos(psi_test);
fprintf('\n=====================================================================\n');
fprintf('LATERAL-CASE CONSISTENCY CHECK (theta=90deg, a=b=0)\n');
fprintf('=====================================================================\n');
fprintf('r_L-r_R via general oblique formula = %.6f m\n', rL_minus_rR_viaGeneral);
fprintf('r_L-r_R via closed form -2e/cos(psi) = %.6f m   (should match exactly)\n', rL_minus_rR_closedForm);
fprintf('difference                          = %.2e m\n', abs(rL_minus_rR_viaGeneral - rL_minus_rR_closedForm));

% ============================== figure ==============================
fig = figure('Name', 'Oblique b_F linearization validation', 'Color', 'w', 'Position', [60 40 1300 800]);

subplot(2,3,1);
hold on; grid on; box on;
plot(e_range*100, bF_exact_vs_e*100, 'k-', 'LineWidth', 1.6, 'DisplayName', 'exact');
plot(e_range*100, bF_lin_vs_e*100, 'r--', 'LineWidth', 1.6, 'DisplayName', 'linear');
xlabel('e (cm)'); ylabel('b_F (cm)'); title('b_F vs e (\psi=0)');
legend('Location', 'best');

subplot(2,3,4);
hold on; grid on; box on;
plot(e_range*100, (bF_exact_vs_e-bF_lin_vs_e)*1000, 'b-', 'LineWidth', 1.4);
xlabel('e (cm)'); ylabel('error (mm)'); title('b_F error vs e (\psi=0)');

subplot(2,3,2);
hold on; grid on; box on;
plot(rad2deg(psi_range), bF_exact_vs_psi*100, 'k-', 'LineWidth', 1.6, 'DisplayName', 'exact');
plot(rad2deg(psi_range), bF_lin_vs_psi*100, 'r--', 'LineWidth', 1.6, 'DisplayName', 'linear');
xlabel('\psi (deg)'); ylabel('b_F (cm)'); title('b_F vs \psi (e=0)');
legend('Location', 'best');

subplot(2,3,5);
hold on; grid on; box on;
plot(rad2deg(psi_range), (bF_exact_vs_psi-bF_lin_vs_psi)*1000, 'b-', 'LineWidth', 1.4);
xlabel('\psi (deg)'); ylabel('error (mm)'); title('b_F error vs \psi (e=0)');

subplot(2,3,[3 6]);
imagesc(e_range*100, rad2deg(psi_range), relErr); axis xy;
colormap(gca, parula); cb = colorbar; ylabel(cb, 'b_f^{*} (%)');
caxis([0 20]);
hold on;
contour(e_range*100, rad2deg(psi_range), relErr, [5 5], 'w-', 'LineWidth', 2);
xlabel('e (cm)'); ylabel('\psi (deg)');
title('b_F relative error (%), white contour = 5% boundary');

sgtitle(sprintf('Oblique front-fan b_F linearization validation (\\theta=%.1f^\\circ, a=%.3fm, b=%.3fm, L_\\theta=%.3fm, W=%.2fm)', ...
    fanDeg, a, b, L_theta, W));

figPath = fullfile(pwd, 'validate_oblique_bF_linearization.png');
exportgraphics(fig, figPath, 'Resolution', 200);
fprintf('\nSaved figure to %s\n', figPath);
end

% =========================================================================
function bF = bF_exact(e, psi, a, b, theta, W)
rFL = (W/2 - e - a*sin(psi) - b*cos(psi)) / sin(theta + psi);
rFR = (W/2 + e + a*sin(psi) - b*cos(psi)) / sin(theta - psi);
bF = rFL - rFR;
end

function bF = bF_linear(e, psi, theta, L_theta)
bF = -(2/sin(theta)) * (e + L_theta*psi);
end

function c = cot_(x)
c = cos(x) / sin(x);
end
