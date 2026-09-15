function [cmd, mem] = ctrl_wall_extend_ref_mixed_opt1_sym21(t, state, ranges, goal, env, sim, mem)
%CTRL_WALL_EXTEND_REF_MIXED_OPT1_SYM2
% WELR navigator for mixed H / ASC / DESC tunnels with persistent virtual-forward remap.
%
% Modifications:
%   1) Cardinal yaw lock is used in confirmed STRAIGHT HORIZONTAL tunnels
%      and confirmed STRAIGHT VERTICAL tunnels.
%   2) Yaw lock is OFF in corners and OFF during vertical->horizontal transition.
%   3) Vertical->horizontal transition no longer ends by time.
%      It ends only when the UAV detects that it is already inside the
%      horizontal tunnel.
%   4) After vertical->H, the selected horizontal direction mem.hdir is kept
%      persistently (no timed unlock). This preserves the chosen corridor
%      direction without requiring an immediate yaw turn.

p = getP(sim);

% -------------------- memory init --------------------
if ~isfield(mem,'rF_hist'),      mem.rF_hist = nan(p.wall_hist_len,1); end
if ~isfield(mem,'hist_i'),       mem.hist_i  = 0; end
if ~isfield(mem,'wall_conf'),    mem.wall_conf = 0; end
if ~isfield(mem,'wall_mode'),    mem.wall_mode = false; end
if ~isfield(mem,'corner_mode'),  mem.corner_mode = false; end
if ~isfield(mem,'turn_dir'),     mem.turn_dir = 0; end
if ~isfield(mem,'pos_ref'),      mem.pos_ref = [state.x; state.y]; end
if ~isfield(mem,'t_ref'),        mem.t_ref = t; end
if ~isfield(mem,'stuck_count'),  mem.stuck_count = 0; end
if ~isfield(mem,'last_cmd'),     mem.last_cmd = [0;0;0;0]; end

% --- nav mode memory ---
if ~isfield(mem,'nav_mode'),      mem.nav_mode = 'H'; end
if ~isfield(mem,'nav_mode_prev'), mem.nav_mode_prev = mem.nav_mode; end

% --- persistent horizontal direction selector ---
if ~isfield(mem,'hdir'), mem.hdir = 'F'; end   % 'F','L','R','B'

% --- transition memory ---
if ~isfield(mem,'trans_on'), mem.trans_on = false; end
if ~isfield(mem,'vz_hold'),  mem.vz_hold = 0; end

% --- vertical commitment memory ---
if ~isfield(mem,'v_t0'), mem.v_t0 = -Inf; end
if ~isfield(mem,'v_z0'), mem.v_z0 = state.z; end

% --- yaw lock memory ---
if ~isfield(mem,'yaw_locked'),         mem.yaw_locked = false; end
if ~isfield(mem,'yaw_lock_ref'),       mem.yaw_lock_ref = 0; end
if ~isfield(mem,'in_straight_H'),      mem.in_straight_H = false; end
if ~isfield(mem,'in_straight_V'),      mem.in_straight_V = false; end

% --- last-valid-reading memory (dropout/invalid hold, not r_inf) ---
if ~isfield(mem,'last_valid')
    mem.last_valid = struct('rF',p.d_safe_front,'rB',p.d_safe_front, ...
                             'rL',p.d_safe_front,'rR',p.d_safe_front, ...
                             'rU',p.vert_enter_open,'rD',p.vert_enter_open);
end
% Per-channel "when was this last a FRESH (non-held) reading" -- lets
% the final reflex clamp widen its margin by how long it's actually been
% flying blind on a channel, not just whether the current sample happens
% to be valid. See dmin_eff below for why: at v_cruise ~0.16 m/s, even a
% short 0.2 s dropout streak is ~3 cm of unseen travel, which is not
% negligible against this controller's ~4 cm nominal (d_min vs
% robot_radius) margin.
if ~isfield(mem,'fresh_t')
    mem.fresh_t = struct('rF',t,'rB',t,'rL',t,'rR',t,'rU',t,'rD',t);
end

% --- closing-rate / time-to-contact history buffer ---
if ~isfield(mem,'ttc_hist') || numel(mem.ttc_hist) ~= p.ttc_hist_len
    mem.ttc_hist = nan(p.ttc_hist_len,1);
    mem.ttc_hist_i = 0;
end

% -------------------- unpack ranges (raw) --------------------
dmin = p.d_min;

rF0 = ranges.rF; rB0 = ranges.rB; rL0 = ranges.rL; rR0 = ranges.rR;
if isfield(ranges,'rU'), rU0 = ranges.rU; else, rU0 = p.r_inf; end
if isfield(ranges,'rD'), rD0 = ranges.rD; else, rD0 = p.r_inf; end

% Hold-last-valid on a dropout/invalid (NaN) reading instead of
% assuming max range ("clear"). A sensor that doesn't know is not the
% same as a sensor that has confirmed open space -- see the
% apply_range_noise.m header for the failure mode this replaces. Also
% tracks staleness (mem.fresh_t) -- see dmin_eff below.
% Ablation: p.enable_reflex_safety=false reverts to the ORIGINAL
% pre-improvement behavior (dropout -> optimistic r_inf "clear").
if p.enable_reflex_safety
    [rF0, mem.last_valid.rF, mem.fresh_t.rF] = holdIfInvalid(rF0, mem.last_valid.rF, mem.fresh_t.rF, t);
    [rB0, mem.last_valid.rB, mem.fresh_t.rB] = holdIfInvalid(rB0, mem.last_valid.rB, mem.fresh_t.rB, t);
    [rL0, mem.last_valid.rL, mem.fresh_t.rL] = holdIfInvalid(rL0, mem.last_valid.rL, mem.fresh_t.rL, t);
    [rR0, mem.last_valid.rR, mem.fresh_t.rR] = holdIfInvalid(rR0, mem.last_valid.rR, mem.fresh_t.rR, t);
    if isfield(ranges,'rU')
        [rU0, mem.last_valid.rU, mem.fresh_t.rU] = holdIfInvalid(rU0, mem.last_valid.rU, mem.fresh_t.rU, t);
    end
    if isfield(ranges,'rD')
        [rD0, mem.last_valid.rD, mem.fresh_t.rD] = holdIfInvalid(rD0, mem.last_valid.rD, mem.fresh_t.rD, t);
    end
else
    if ~isfinite(rF0), rF0 = p.r_inf; end
    if ~isfinite(rB0), rB0 = p.r_inf; end
    if ~isfinite(rL0), rL0 = p.r_inf; end
    if ~isfinite(rR0), rR0 = p.r_inf; end
    if ~isfinite(rU0), rU0 = p.r_inf; end
    if ~isfinite(rD0), rD0 = p.r_inf; end
end

% -------------------- navigation mode selection --------------------
mem.nav_mode_prev = mem.nav_mode;

front_tight = (rF0 < p.vert_enter_front);

up_ok   = (rU0 > p.vert_enter_open) && ((rU0 - max(rL0,rR0)) > p.vert_side_diff);
down_ok = (rD0 > p.vert_enter_open) && ((rD0 - max(rL0,rR0)) > p.vert_side_diff);

commit_ok = true;
if ~strcmp(mem.nav_mode,'H')
    min_time_ok = (t - mem.v_t0) > p.vert_min_time;
    min_dz_ok   = abs(state.z - mem.v_z0) > p.vert_min_dz;
    commit_ok   = (min_time_ok || min_dz_ok);
end

if strcmp(mem.nav_mode,'H')
    if front_tight
        if up_ok && ~down_ok
            mem.nav_mode = 'ASC';
        elseif down_ok && ~up_ok
            mem.nav_mode = 'DESC';
        elseif up_ok && down_ok
            v_intent = -mem.last_cmd(3);

            if (v_intent < -p.vert_intent_eps) && (rD0 > p.vert_intent_clear)
                mem.nav_mode = 'DESC';
            elseif (v_intent > +p.vert_intent_eps) && (rU0 > p.vert_intent_clear)
                mem.nav_mode = 'ASC';
            else
                if (rD0 > rU0 + p.vert_choice_eps)
                    mem.nav_mode = 'DESC';
                else
                    mem.nav_mode = 'ASC';
                end
            end
        end

        if ~strcmp(mem.nav_mode,'H')
            mem.v_t0 = t;
            mem.v_z0 = state.z;

            mem.trans_on = false;
            mem.corner_mode = false;
            mem.wall_mode = false;
            mem.wall_conf = 0;
            mem.stuck_count = 0;

            mem.yaw_locked = false;
            mem.in_straight_H = false;
            mem.in_straight_V = false;

            mem.ttc_hist(:) = nan;
            mem.ttc_hist_i = 0;
        end
    end

elseif strcmp(mem.nav_mode,'ASC')
    valsH = [rF0, rL0, rR0, rB0];
    [maxH, idxMax] = max(valsH);
    horiz_open = (maxH > p.horiz_open_thresh);

    if commit_ok && (rU0 < p.vert_exit_front) && horiz_open
        mem.nav_mode = 'H';

        switch idxMax
            case 1, mem.hdir = 'F';
            case 2, mem.hdir = 'L';
            case 3, mem.hdir = 'R';
            case 4, mem.hdir = 'B';
        end

        mem.trans_on = true;
        mem.vz_hold  = mem.last_cmd(3);

        mem.corner_mode = false;
        mem.wall_mode   = false;
        mem.wall_conf   = 0;
        mem.stuck_count = 0;

        mem.yaw_locked = false;
        mem.in_straight_H = false;
        mem.in_straight_V = false;

        mem.ttc_hist(:) = nan;
        mem.ttc_hist_i = 0;
    end

elseif strcmp(mem.nav_mode,'DESC')
    valsH = [rF0, rL0, rR0, rB0];
    [maxH, idxMax] = max(valsH);
    horiz_open = (maxH > p.horiz_open_thresh);

    if commit_ok && (rD0 < p.vert_exit_front) && horiz_open
        mem.nav_mode = 'H';

        switch idxMax
            case 1, mem.hdir = 'F';
            case 2, mem.hdir = 'L';
            case 3, mem.hdir = 'R';
            case 4, mem.hdir = 'B';
        end

        mem.trans_on = true;
        mem.vz_hold  = mem.last_cmd(3);

        mem.corner_mode = false;
        mem.wall_mode   = false;
        mem.wall_conf   = 0;
        mem.stuck_count = 0;

        mem.yaw_locked = false;
        mem.in_straight_H = false;
        mem.in_straight_V = false;

        mem.ttc_hist(:) = nan;
        mem.ttc_hist_i = 0;
    end
end

% -------------------- remap ranges to virtual navigation frame --------------------
switch mem.nav_mode
    case 'ASC'
        % virtual forward = up
        rF = rU0;  rB = rD0;
        rL = rL0;  rR = rR0;

    case 'DESC'
        % virtual forward = down
        rF = rD0;  rB = rU0;
        rL = rL0;  rR = rR0;

    otherwise
        switch mem.hdir
            case 'F'
                rF = rF0; rB = rB0; rL = rL0; rR = rR0;
            case 'L'
                rF = rL0; rB = rR0; rL = rB0; rR = rF0;
            case 'R'
                rF = rR0; rB = rL0; rL = rF0; rR = rB0;
            case 'B'
                rF = rB0; rB = rF0; rL = rR0; rR = rL0;
            otherwise
                rF = rF0; rB = rB0; rL = rL0; rR = rR0;
        end
end

% -------------------- safety envelope (virtual frame) --------------------
vx_ub = p.k_safe * max(rF - dmin, 0);
vx_lb = -p.k_safe * max(rB - dmin, 0);
vy_ub = p.k_safe * max(rL - dmin, 0);
vy_lb = -p.k_safe * max(rR - dmin, 0);

% -------------------- goal distance --------------------
dist_goal = hypot(goal.x - state.x, goal.y - state.y);

% -------------------- update rF history + wall confidence --------------------
mem.hist_i = mem.hist_i + 1;
idx = mod(mem.hist_i-1, p.wall_hist_len) + 1;
mem.rF_hist(idx) = rF;

rF_valid = mem.rF_hist(isfinite(mem.rF_hist));
if numel(rF_valid) >= max(3, ceil(p.wall_hist_len/2))
    rF_mean = mean(rF_valid);
    rF_std  = std(rF_valid);
else
    rF_mean = rF;
    rF_std  = Inf;
end

is_front_active = (rF_mean < p.wall_active_max) && (rF_mean > (dmin + p.front_active_min_margin));
is_front_stable = (rF_std < p.wall_std_th);
wall_suggest = is_front_active && is_front_stable;

if wall_suggest
    mem.wall_conf = min(mem.wall_conf + 1, p.wall_conf_max);
else
    mem.wall_conf = max(mem.wall_conf - 1, -p.wall_conf_max);
end

if ~mem.wall_mode
    if mem.wall_conf >= p.wall_conf_enter
        mem.wall_mode = true;
    end
else
    if mem.wall_conf <= p.wall_conf_exit
        mem.wall_mode = false;
    end
end

% -------------------- closing-rate / time-to-contact (TTC) --------------------
% Onboard-mirroring predictive brake: a short ring buffer of the
% (already remapped) virtual-forward range rF estimates how fast it is
% closing, so a fast approach gets braked BEFORE rF alone crosses
% p.d_safe_front/dmin -- not just react to instantaneous distance. Same
% mechanism as closingRate/ttc in the companion firmware apps
% (app_cf21_slam_tunnel.c / app_cf_brushless_slam_tunnel.c). Buffer is
% reset on every nav_mode transition (see those blocks above) since rF's
% meaning changes discontinuously there, not because of a real approach.
% Ablation: p.enable_memory_ttc=false reverts to no closing-rate
% awareness at all (gate_ttc=1, i.e. a pure no-op multiplier below).
if p.enable_memory_ttc
    mem.ttc_hist_i = mem.ttc_hist_i + 1;
    ttc_idx = mod(mem.ttc_hist_i-1, p.ttc_hist_len) + 1;
    oldest_rF = mem.ttc_hist(ttc_idx);
    mem.ttc_hist(ttc_idx) = rF;

    closing_rate = 0;
    ttc = Inf;
    if isfinite(oldest_rF) && mem.ttc_hist_i > p.ttc_hist_len
        window_s = p.ttc_hist_len * sim.dt;
        closing_rate = (oldest_rF - rF) / window_s;
        if closing_rate > p.ttc_rate_eps
            ttc = rF / closing_rate;
        end
    end
    gate_ttc = local_gate(ttc, p.ttc_stop_s, p.ttc_slow_s);
else
    gate_ttc = 1;
end

% Suppress TTC during wall_mode/corner_mode: both already have their own
% tuned, DISTANCE-based slowdown (wall_mode's carrot x_carrot=clamp(rF-
% d_safe_front,...), corner_mode's creep probe) -- TTC firing on top of
% an already-decelerating INTENTIONAL maneuver adds hesitation, not
% safety. TTC exists to catch a fast approach BEFORE those distance
% responses would engage, not to duplicate them once they're already
% active -- this is what let widening ttc_stop_s/ttc_slow_s (for
% genuine early-warning reach in the nominal-cruise case) fight normal
% cornering instead. mem.corner_mode here is one tick stale (this
% tick's corner-mode decision hasn't been made yet at this point in the
% function) -- fine, since corner_mode doesn't flip every tick.
if mem.wall_mode || mem.corner_mode
    gate_ttc = 1;
end

% -------------------- stuck detection --------------------
if (t - mem.t_ref) >= p.stuck_window
    dp = hypot(state.x - mem.pos_ref(1), state.y - mem.pos_ref(2));
    mem.pos_ref = [state.x; state.y];
    mem.t_ref = t;

    if (dist_goal > p.d_goal_stop) && (dp < p.stuck_progress_min)
        mem.stuck_count = min(mem.stuck_count + 1, 50);
    else
        mem.stuck_count = max(mem.stuck_count - 1, 0);
    end
end

% -------------------- corner mode --------------------
% Suppress corners during vertical->H transition so remapped progress dominates.
if mem.trans_on
    mem.corner_mode = false;
else
    if ~mem.corner_mode
        if (rF < p.corner_enter) || (mem.stuck_count >= p.stuck_count_enter)
            mem.corner_mode = true;

            if abs(rL - rR) < p.tie_eps
                if strcmp(mem.nav_mode,'H')
                    theta_g = atan2(goal.y - state.y, goal.x - state.x);
                    e_yaw = util_wrapToPi(theta_g - state.yaw);
                    mem.turn_dir = sign(e_yaw);
                    if mem.turn_dir == 0, mem.turn_dir = +1; end
                else
                    mem.turn_dir = (rL >= rR)*2 - 1;
                    if mem.turn_dir == 0, mem.turn_dir = +1; end
                end
            else
                mem.turn_dir = (rL >= rR)*2 - 1;
            end

            mem.yaw_locked = false;
            mem.in_straight_H = false;
            mem.in_straight_V = false;
        end
    else
        if (rF > p.corner_exit) && (mem.stuck_count <= p.stuck_count_exit)
            mem.corner_mode = false;
        end
    end
end

% -------------------- straight tunnel detection --------------------
% Horizontal straight uses remapped H frame. Vertical straight uses ASC/DESC frame.
straight_H_now = strcmp(mem.nav_mode,'H') && ...
                 (~mem.corner_mode) && ...
                 (~mem.trans_on) && ...
                 (rF > p.horiz_straight_front_min) && ...
                 (rL < p.wall_seen_max) && ...
                 (rR < p.wall_seen_max);

straight_V_now = strcmp(mem.nav_mode,'H') && ...
                 (rF0 < p.wall_seen_max) && ...
                 (rB0 < p.wall_seen_max) && ...
                 (rL0 < p.wall_seen_max) && ...
                 (rR0 < p.wall_seen_max);
straight_HV_now = strcmp(mem.nav_mode,'H') && ...
                (rU0 < p.wall_seen_max) && ...
                 (rD0 < p.wall_seen_max) && ...
                 (rL0 < p.wall_seen_max) && ...
                 (rR0 < p.wall_seen_max);

entered_straight_H = straight_H_now && ~mem.in_straight_H;
left_straight_H    = ~straight_H_now && mem.in_straight_H;

entered_straight_V = straight_V_now && ~mem.in_straight_V;
left_straight_V    = ~straight_V_now && mem.in_straight_V;

if entered_straight_H
    mem.in_straight_H = true;
    mem.in_straight_V = false;
    mem.yaw_lock_ref = local_nearest_cardinal(state.yaw);
    mem.yaw_locked = true;
elseif left_straight_H
    mem.in_straight_H = false;
    mem.yaw_locked = false;
end

if entered_straight_V
    mem.in_straight_V = true;
    mem.in_straight_H = false;
    mem.yaw_lock_ref = local_nearest_cardinal(state.yaw);
    mem.yaw_locked = true;
elseif left_straight_V
    mem.in_straight_V = false;
    mem.yaw_locked = false;
end

% -------------------- end transition ONLY by horizontal-tunnel detection --------------------
if mem.trans_on && straight_HV_now
    mem.trans_on = false;
    mem.yaw_lock_ref = local_nearest_cardinal(state.yaw);
    mem.yaw_locked = true;
    mem.in_straight_H = true;
end

% -------------------- build local reference in virtual frame --------------------
vx = 0; vy = 0; wz = 0;
wz_base = 0;

if dist_goal < p.d_goal_stop
    vx = 0; vy = 0; wz = 0;
    wz_base = 0;

else
    y_center = p.k_center * (rL - rR);
    if p.use_center_norm
        y_center = p.k_center * (rL - rR) / (rL + rR + p.eps);
    end
    y_center = util_clamp(y_center, -p.y_center_max, p.y_center_max);

    if mem.corner_mode
        vy_ref = mem.turn_dir * p.vy_creep;
        vx_ref = 0;
        if rF > (dmin + p.forward_probe_margin)
            vx_ref = min(p.v_probe, p.k_forward * max(rF - dmin, 0));
        end
        wz_ref = mem.turn_dir * p.wz_turn_slow;

        vx = vx_ref;
        vy = vy_ref;
        wz = wz_ref;
        wz_base = wz_ref;

    elseif mem.wall_mode
        s = util_clamp(rF - p.d_safe_front, 0, p.s_max);

        x_carrot = s;
        y_carrot = y_center;

        psi  = atan2(y_carrot, max(x_carrot, p.eps));
        vmag = util_clamp(p.k_carrot * x_carrot, 0, p.v_cruise);

        vx = vmag * cos(psi);
        vy = vmag * sin(psi);

        wz_base = util_clamp(p.k_yaw_to_carrot * psi, -p.wz_max, p.wz_max);
        wz = wz_base;

        if rF < p.slowdown_front
            vx = min(vx, p.v_slow);
        end

    else
        if strcmp(mem.nav_mode,'H')
            theta_g = atan2(goal.y - state.y, goal.x - state.x);
            e_yaw = util_wrapToPi(theta_g - state.yaw);
            wz_base = util_clamp(p.k_yaw * e_yaw, -p.wz_max, p.wz_max) * p.wz_goal_scale;
        else
            % In vertical tunnels, avoid goal-yaw steering; keep yaw neutral unless locked
            wz_base = 0;
        end

        vx = min(p.v_cruise, p.k_forward * max(rF - dmin, 0));
        vy = y_center;
        wz = wz_base;
    end
end

% TTC predictive brake: scales only the forward-progress component
% (whatever direction is currently virtual-forward -- H, ASC, or DESC),
% same as the firmware's gate*vCruise. Lateral/turn commands are
% untouched, matching the firmware design (only forward speed is gated).
vx = vx * gate_ttc;

% -------------------- yaw selection --------------------
% Corner: free yaw (already assigned)
% Transition: no yaw after vertical->H
% Straight H/V: cardinal yaw lock blended with base yaw
if mem.corner_mode
    % keep corner yaw
elseif mem.trans_on
    wz = 0;
elseif mem.yaw_locked && (mem.in_straight_H || mem.in_straight_V)
    e_yaw_lock = util_wrapToPi(mem.yaw_lock_ref - state.yaw);
    wz_lock = util_clamp(p.k_yaw_hold * e_yaw_lock, ...
                         -p.wz_cardinal_max, p.wz_cardinal_max);

    if abs(e_yaw_lock) < p.yaw_deadband
        wz_lock = 0;
    end

    wz = util_clamp(wz_base + wz_lock, -p.wz_max, p.wz_max);
else
    wz = wz_base;
end

% -------------------- clamp in virtual frame --------------------
vx = min(max(vx, vx_lb), vx_ub);
vy = min(max(vy, vy_lb), vy_ub);
wz = util_clamp(wz, -p.wz_max, p.wz_max);

if (hypot(vx,vy) < 0.01) && (dist_goal > p.d_goal_stop)
    mem.stuck_count = min(mem.stuck_count + 1, 50);
end

% -------------------- altitude / ceiling (raw frame) --------------------
vz_z = util_clamp(p.k_z*(goal.z - state.z), -p.vz_max, p.vz_max);
if isfinite(rU0) && (rU0 < p.d_ceiling)
    vz_z = min(vz_z,0) - p.k_ceiling*(p.d_ceiling - rU0);
end
vz_z = util_clamp(vz_z, -p.vz_max, p.vz_max);

% -------------------- map virtual-frame motion back to raw frame --------------------
vx_nav = vx;
vy_nav = vy;
vz = vz_z;

if strcmp(mem.nav_mode,'ASC')
    vz = +vx_nav;
    vx = vz_z;
    vy = vy_nav;

elseif strcmp(mem.nav_mode,'DESC')
    vz = -vx_nav;
    vx = vz_z;
    vy = vy_nav;

else
    switch mem.hdir
        case 'F'
            vx = vx_nav;   vy = vy_nav;
        case 'L'
            vx = -vy_nav;  vy = +vx_nav;
        case 'R'
            vx = +vy_nav;  vy = -vx_nav;
        case 'B'
            vx = -vx_nav;  vy = -vy_nav;
        otherwise
            vx = vx_nav;   vy = vy_nav;
    end
end

% -------------------- optional transition softening without time --------------------
% While transition is active, keep commands conservative until horizontal tunnel is confirmed.
if mem.trans_on
    vx = p.trans_gain_xy * vx;
    vy = p.trans_gain_xy * vy;

    % fade vertical residual immediately by fixed gain, not by time
    vz = p.trans_gain_vz * mem.vz_hold + (1 - p.trans_gain_vz) * vz;
end

% -------------------- final raw-frame safety clamping --------------------
% Hard reflex layer: overrides whatever nav_mode/corner/wall-follow logic
% produced above, using only the raw (unremapped) instantaneous ranges.
% Four additions beyond the original per-axis clamp:
%   1) Speed-scaled margin: dmin_eff grows with the PREVIOUS commanded
%      speed (braking-distance style), so cruising fast reserves more
%      standoff than creeping does. Applied only here (the last-line-of-
%      defense clamp), not in the soft planning envelope above, so the
%      wall-follow/corner gains upstream don't need re-tuning.
%   2) Staleness-scaled margin: dmin_eff ALSO grows with how long it's
%      been since the xy-relevant channels (rF0/rB0/rL0/rR0) were last a
%      fresh (non-held) reading, times current speed -- i.e. the
%      worst-case distance travelled "blind" during a dropout streak.
%      Hold-last-valid alone (see holdIfInvalid) fixes the WRONG-
%      DIRECTION failure (stale read as "confirmed clear"), but a stale
%      read is still stale; at this controller's speeds even a 0.2 s
%      streak is centimeters of unseen travel, non-negligible against
%      the ~4 cm nominal (d_min vs robot_radius) margin. Monte-Carlo
%      testing under 5% dropout (run_dropout_trials.m) showed near-
%      certain collisions without this term.
%   3) TTC-urgency margin: dmin_eff ALSO grows when the closing-rate/TTC
%      buffer (see gate_ttc above) predicts an imminent approach --
%      proportional to (1-gate_ttc) (0 when TTC says "safe", up to 1 at
%      full TTC-stop urgency) times current speed times p.ttc_stop_s (a
%      distance-equivalent of the TTC-stop time budget). This is what
%      makes the memory/TTC mechanism genuinely COMPLEMENT the reflex
%      layer instead of being a redundant separate cap: previously
%      gate_ttc only ever throttled vx (see its multiply above), so a
%      fast-closing DIAGONAL approach got zero benefit from TTC at all,
%      and the final clamp's min(gate_ttc-reduced-vx, dmin_eff-bound)
%      meant whichever mechanism was already more conservative simply
%      won -- the two never compounded. Feeding TTC urgency into dmin_eff
%      itself means box_speed_limit's diagonal/lateral protection now
%      benefits from the closing-rate prediction too, not just vx.
%   4) Combined-direction (box) clamp: bounds |[vx,vy]| by the clearance
%      in the ACTUAL direction of travel, not just per-axis -- see
%      box_speed_limit() below for why per-axis alone under-constrains a
%      diagonal command approaching a corner.
% Ablation: p.enable_reflex_safety=false reverts dmin_eff to the plain
% constant dmin (no speed/staleness/TTC scaling) and skips
% box_speed_limit, leaving only the ORIGINAL independent per-axis clamp.
if p.enable_reflex_safety
    v_now = hypot(mem.last_cmd(1), mem.last_cmd(2));
    staleness_xy = max([t - mem.fresh_t.rF, t - mem.fresh_t.rB, t - mem.fresh_t.rL, t - mem.fresh_t.rR]);
    ttc_margin = p.k_ttc_margin * (1 - gate_ttc) * v_now * p.ttc_stop_s;
    dmin_eff = dmin + (v_now^2) / (2 * p.a_max_safety) + v_now * staleness_xy + ttc_margin;
else
    dmin_eff = dmin;
end

vx_ub_raw = p.k_safe    * max(rF0 - dmin_eff, 0);
vx_lb_raw = -p.k_safe   * max(rB0 - dmin_eff, 0);
vy_ub_raw = p.k_safe    * max(rL0 - dmin_eff, 0);
vy_lb_raw = -p.k_safe   * max(rR0 - dmin_eff, 0);
vz_ub_raw = p.k_safe_z  * max(rU0 - dmin_eff, 0);
vz_lb_raw = -p.k_safe_z * max(rD0 - dmin_eff, 0);

vx = min(max(vx, vx_lb_raw), vx_ub_raw);
vy = min(max(vy, vy_lb_raw), vy_ub_raw);
vz = min(max(vz, vz_lb_raw), vz_ub_raw);

if p.enable_reflex_safety
    [vx, vy] = box_speed_limit(vx, vy, rF0, rB0, rL0, rR0, dmin_eff, p.k_safe);
end

% -------------------- debug --------------------
if p.debug_print && ~strcmp(mem.nav_mode, mem.nav_mode_prev)
    fprintf(['[MODE CHANGE] t=%.2f  %s -> %s | ' ...
             'last_wz=%.2f | pos=(%.2f, %.2f, %.2f) yaw=%.2f | ' ...
             'rawF=%.2f rawL=%.2f rawR=%.2f rawU=%.2f rawD=%.2f | ' ...
             'virtF=%.2f virtL=%.2f virtR=%.2f | hdir=%s trans=%d\n'], ...
             t, mem.nav_mode_prev, mem.nav_mode, ...
             mem.last_cmd(3), ...
             state.x, state.y, state.z, state.yaw, ...
             rF0, rL0, rR0, rU0, rD0, ...
             rF, rL, rR, mem.hdir, mem.trans_on);
end

mem.last_cmd = [vx; vy; vz; wz];
cmd = struct('vx',vx,'vy',vy,'vz',vz,'wz',wz);

end

% =====================================================================
function p = getP(sim)
if isfield(sim,'wall_extend_params')
    p = sim.wall_extend_params;
    return;
end

% --- debug ---
p.debug_print = true;   % set false (e.g. via sim.bug_params) to silence [MODE CHANGE] fprintf -- large batch/Monte-Carlo runs otherwise spend real time on console I/O

% --- ablation switches (both default true = full behavior) ---
p.enable_reflex_safety = true;   % hold-last-valid + speed/staleness-scaled dmin_eff + box_speed_limit clamp -- see the "Hard reflex layer" block below
p.enable_memory_ttc    = true;   % closing-rate/TTC history buffer + predictive braking gate -- see the "closing-rate / time-to-contact" block below

% --- safety / envelope ---
p.d_min  = 0.12;
p.k_safe = 1.2;
p.k_safe_z = p.k_safe;
p.r_inf  = 10.0;

% --- speed-scaled stopping margin (final reflex clamp only) ---
p.a_max_safety = 1.0;   % m/s^2, assumed max deceleration for dmin_eff = dmin + v^2/(2*a_max_safety)

% --- closing-rate / time-to-contact (TTC) braking ---
% ttc_stop_s/ttc_slow_s widened from the original 0.6/1.5 s: at this
% controller's v_cruise (~0.16 m/s), 0.6 s of head-on closing is only
% ~10 cm -- by then the reflex layer's own dmin_eff-based margin is
% already binding, so TTC had almost no "early warning" room to add
% anything beyond it. Widening gives TTC genuine reach beyond the
% instantaneous-distance margin. Not so wide that it fights the
% INTENTIONAL wall_mode approach-and-follow slowdown (which already
% ramps vx down via k_carrot as rF shrinks toward d_safe_front) --
% re-tune here if empirical results (run_dropout_xy_trajectories_*.m)
% show it's now over-braking during normal cornering.
p.ttc_window_s = 0.3;    % s, closing-rate lookback window
p.ttc_stop_s   = 0.9;    % s, time-to-contact -> full forward-speed cutoff
p.ttc_slow_s   = 2.2;    % s, time-to-contact -> starts slowing from full speed
p.ttc_rate_eps = 0.001;  % m/s, minimum closing rate to bother computing a finite TTC
p.k_ttc_margin = 1.0;    % gain on how much TTC urgency (1-gate_ttc) adds to dmin_eff -- see the final reflex clamp below
if isfield(sim,'dt') && sim.dt > 0
    p.ttc_hist_len = max(3, round(p.ttc_window_s / sim.dt));
else
    p.ttc_hist_len = 30;
end

% --- vertical tunnel mode switching ---
p.vert_enter_front    = 0.22;
p.vert_enter_open     = 0.30;
p.vert_side_diff      = 0.08;
p.vert_exit_front     = 0.22;
p.horiz_open_thresh   = p.vert_enter_open;

p.vert_choice_eps   = 0.05;
p.vert_min_time     = 1.0;
p.vert_min_dz       = 0.20;
p.vert_intent_eps   = 0.03;
p.vert_intent_clear = 0.30;

% --- straight tunnel detection ---
p.wall_seen_max            = 0.60;
p.wall_seen_max1            = 0.5;
p.horiz_straight_front_min = 0.32;
p.vert_straight_front_min  = 0.32;

% --- local reference (wall-extend) ---
p.d_safe_front   = 0.18;
p.s_max          = 0.35;
p.k_carrot       = 1.5;
p.v_cruise       = 0.16;
p.v_slow         = 0.06;
p.slowdown_front = 0.30;

% --- centering ---
p.k_center        = 0.9;
p.use_center_norm = false;
p.y_center_max    = 0.10;
p.eps             = 1e-6;

% --- wall detection / confidence ---
p.wall_hist_len           = 8;
p.wall_std_th             = 0.03;
p.wall_active_max         = 0.80;
p.front_active_min_margin = 0.02;
p.wall_conf_max           = 20;
p.wall_conf_enter         = 5;
p.wall_conf_exit          = 0;

% --- corner mode ---
p.corner_enter = 0.24;
p.corner_exit  = 0.34;
p.tie_eps      = 0.02;

p.vy_creep             = 0.04;
p.v_probe              = 0.05;
p.forward_probe_margin = 0.04;
p.wz_turn_slow         = 0.6;

% --- yaw ---
p.wz_max          = 1.2;
p.k_yaw           = 0.6;
p.wz_goal_scale   = 0.5;
p.k_yaw_to_carrot = 0.7;

% cardinal yaw lock
p.k_yaw_hold      = 0.8;
p.wz_cardinal_max = 0.25;
p.yaw_deadband    = deg2rad(4);

% --- forward gain for nominal ---
p.k_forward = 1.0;

% --- stuck detector ---
p.stuck_window       = 1.0;
p.stuck_progress_min = 0.03;
p.stuck_count_enter  = 3;
p.stuck_count_exit   = 1;

% --- goal stop ---
p.d_goal_stop = 0.12;

% --- altitude ---
p.k_z = 1.0;
p.vz_max = 0.25;
p.d_ceiling = 0.12;
p.k_ceiling = 1.0;

% --- condition-based transition softening ---
p.trans_gain_xy = 0.5;   % conservative H command during vertical->H transition
p.trans_gain_vz = 0.5;   % keep part of previous vertical command during transition

if isfield(sim,'bug_params')
    f = fieldnames(sim.bug_params);
    for i = 1:numel(f)
        p.(f{i}) = sim.bug_params.(f{i});
    end
end
end

% =====================================================================
function a = util_clamp(a, lo, hi)
a = min(max(a, lo), hi);
end

function ang = util_wrapToPi(ang)
ang = mod(ang + pi, 2*pi) - pi;
end

function ang_ref = local_nearest_cardinal(yaw)
cards = [0, pi/2, pi, -pi/2];
err = arrayfun(@(a) abs(util_wrapToPi(yaw - a)), cards);
[~, idx] = min(err);
ang_ref = util_wrapToPi(cards(idx));
end

% =====================================================================
function [r_out, last_valid_out, fresh_t_out] = holdIfInvalid(r_in, last_valid_in, fresh_t_in, t)
% Sample-and-hold fallback for a dropout/invalid (NaN/Inf) reading:
% return the last known-valid value instead of an optimistic "clear".
% Also tracks the timestamp of the last FRESH reading, so the caller can
% tell not just "is this held" but "for how long" -- see dmin_eff.
if isfinite(r_in)
    r_out = r_in;
    last_valid_out = r_in;
    fresh_t_out = t;
else
    r_out = last_valid_in;
    last_valid_out = last_valid_in;
    fresh_t_out = fresh_t_in;
end
end

% =====================================================================
function [vx, vy] = box_speed_limit(vx, vy, rF, rB, rL, rR, dmin, k_safe)
% Bound the combined [vx,vy] vector by the clearance in its OWN
% direction of travel (rectangle/box safety region from the 4 cardinal
% beams), not just per-axis. For direction (cosT,sinT), the box
% boundary distance along that ray is 1/(|cosT|/r_x + |sinT|/r_y), where
% r_x,r_y are the one-sided extents (rF or rB, rL or rR) selected by the
% sign of cosT/sinT. Degenerates to the ordinary per-axis clamp along
% the cardinal directions, so this only ever tightens a diagonal
% command relative to the (already-applied) per-axis clamp -- never
% loosens it.
v = hypot(vx, vy);
if v < 1e-6
    return;
end
cosT = vx / v; sinT = vy / v;

if cosT >= 0, rx = rF; else, rx = rB; end
if sinT >= 0, ry = rL; else, ry = rR; end

denom = abs(cosT)/max(rx - dmin, 1e-6) + abs(sinT)/max(ry - dmin, 1e-6);
v_max = max(k_safe / max(denom, 1e-6), 0);

if v > v_max
    scale = v_max / v;
    vx = vx * scale;
    vy = vy * scale;
end
end

% =====================================================================
function g = local_gate(d, d_stop, d_slow)
% Same ramp shape as util_gate.m / the firmware's localGate(): 0 at/below
% d_stop, 1 at/above d_slow, linear between. Works for a distance (m) OR
% a time-to-contact (s) -- see its call site in this file.
if d <= d_stop
    g = 0;
elseif d >= d_slow
    g = 1;
else
    g = (d - d_stop) / (d_slow - d_stop);
end
end