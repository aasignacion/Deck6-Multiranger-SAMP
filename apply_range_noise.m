function ranges = apply_range_noise(ranges, noiseCfg)
%APPLY_RANGE_NOISE Inject realistic ToF sensor imperfections into a
% ranges struct. Deck-agnostic -- iterates whatever fields are present,
% so the SAME function/config applies unchanged to the 6-beam
% multiranger (raycast_ranges_obbs.m) and the 9-beam vl53l8cx
% (raycast_ranges9_obbs.m). Using the same noise model on both is what
% keeps a noisy comparison fair -- any performance gap should trace to
% sensing richness, not to one deck accidentally being modeled noisier.
%
% Models three practical ToF imperfections:
%   1) Range-dependent Gaussian noise (photon shot noise grows with
%      distance, so std dev = sigma0 + sigma_slope*range)
%   2) Dropouts -- a fraction of readings return no valid target and
%      get reported as max range (angled/low-reflectivity surfaces,
%      out-of-range), matching how the real deck reports "no detection"
%   3) Quantization to whole mm, matching the firmware's uint16_t mm
%      reporting (see sensor_decks.py / vl53l8cx_deck.c in the
%      companion Python project) -- real hardware never reports e.g.
%      1.23456 m, only 1.235 m
%
% noiseCfg fields (all optional; defaults are a reasonable starting
% point for a VL53L1X/L8CX-class sensor, NOT validated against your
% actual hardware's datasheet/measured noise -- tune sigma0/sigma_slope
% once you have real bench data):
%   sigma0        - base std dev (m) at zero range,         default 0.005 (5 mm)
%   sigma_slope   - added std dev per meter of range,       default 0.01  (1% of range)
%   dropout_prob  - probability [0,1] a reading is a dropout, default 0.01
%   max_range     - value reported on dropout (m),          default 4.0 (~VL53L8CX datasheet max range)
%   min_range     - clamp result to at least this (m),      default 0
%
% Usage: enable in a run script by setting, before calling
% simulate_controller/simulate_controller9:
%   sim.sensor_noise = struct('sigma0',0.005,'sigma_slope',0.01,'dropout_prob',0.01,'max_range',env.max_range);
% Leave sim.sensor_noise unset for the original noiseless behavior
% (both simulate_controller.m and simulate_controller9.m only call this
% when sim.sensor_noise is present, so nothing changes unless you opt in).

if nargin < 2, noiseCfg = struct(); end
sigma0       = getf(noiseCfg, 'sigma0', 0.005);
sigma_slope  = getf(noiseCfg, 'sigma_slope', 0.01);
dropout_prob = getf(noiseCfg, 'dropout_prob', 0.01);
min_range    = getf(noiseCfg, 'min_range', 0);
max_range    = getf(noiseCfg, 'max_range', 4.0);   % reported on dropout -- pass env.max_range for consistency with your scenario

fn = fieldnames(ranges);
for i = 1:numel(fn)
    r = ranges.(fn{i});
    if ~isfinite(r)
        continue;   % leave already-invalid/inf readings alone
    end

    if dropout_prob > 0 && rand() < dropout_prob
        r_noisy = max_range;
    else
        sigma = sigma0 + sigma_slope * r;
        r_noisy = r + sigma * randn();
    end

    r_noisy = round(r_noisy * 1000) / 1000;   % quantize to mm
    r_noisy = max(r_noisy, min_range);

    ranges.(fn{i}) = r_noisy;
end
end

function v = getf(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
