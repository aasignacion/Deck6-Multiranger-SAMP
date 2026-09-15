function log = simulate_controller9(controllerFcn, ctrlName, env, sim)
%SIMULATE_CONTROLLER9  Kinematic sim + 9-sensor (vl53l8cx) raycast sensing.
%
% Same contract as simulate_controller.m, but senses via
% raycast_ranges9_obbs.m (9 rays: rFL,rC,rFR,rFU,rFD,rL,rR,rB,rU) instead
% of the 6-ray multiranger model, and logs all 9 channels. Use this with
% a controller written for the 9-sensor deck, e.g.
% ctrl_wall_extend_ref_mixed_opt1_sym21_vl53l8cx9.m.
%
% controllerFcn: @(t, state, ranges, goal, env, sim, mem) -> [cmd, mem]
% cmd: struct('vx','vy','vz','wz') in BODY frame
%
% NOTE: this deck currently has no straight-down sensor (only the
% forward-tilted rFD) -- see the comment in raycast_ranges9_obbs.m.
% infer_mode_and_pF9 below uses rFD as the DOWN-mode progress signal in
% its place; this is an oblique, incomplete view of what's directly
% below and is expected to be less capable at DESC detection than the
% multiranger's straight-down beam until dedicated down sensors are
% added (per the user: "downfront and downback are not used yet").

dt = sim.dt;
N  = floor(sim.T/dt) + 1;

state = sim.init;
goal  = sim.goal;

vlim = sim.v_limits; % [vx vy vz]
wlim = sim.w_limit;

sensor_dt = sim.sensor_dt;
t_next_sense = 0;

if ~isfield(env,'obbs') || isempty(env.obbs)
    if isfield(env,'sdf_path')
        env.obbs = load_sdf_obbs(env.sdf_path);
    else
        error('env.obbs is empty and env.sdf_path not provided.');
    end
end

ranges = raycast_ranges9_obbs(state, env.obbs, env.max_range);
if isfield(sim, 'sensor_noise')
    ranges = apply_range_noise(ranges, sim.sensor_noise);
end

mem = struct();

% ---- allocate logs ----
log.t   = zeros(N,1);
log.x   = zeros(N,1); log.y = zeros(N,1); log.z = zeros(N,1); log.yaw = zeros(N,1);
log.vx  = zeros(N,1); log.vy = zeros(N,1); log.vz = zeros(N,1); log.wz  = zeros(N,1);
log.rFL = zeros(N,1); log.rC  = zeros(N,1); log.rFR = zeros(N,1);
log.rFU = zeros(N,1); log.rFD = zeros(N,1);
log.rL  = zeros(N,1); log.rR  = zeros(N,1); log.rB  = zeros(N,1); log.rU = zeros(N,1);

% derived signals
log.rmin  = zeros(N,1);
log.speed = zeros(N,1);

% mode + virtual front
log.mode = strings(N,1);   % "H","UP","DOWN"
log.pF   = zeros(N,1);     % virtual front range used for progress

% flags
log.collision    = false;
log.goal_reached = false;
log.exited       = false;
log.ctrl         = ctrlName;
log.stop_reason  = "timeout";

RCHAN = {'rFL','rC','rFR','rFU','rFD','rL','rR','rB','rU'};

% ---- optional exit-then-stop detection (OFF unless sim.exit_detect is set) ----
% See simulate_controller.m for the full rationale. Front here is the
% conservative fan minimum min(rFL,rC,rFR) -- same convention as the
% brushless firmware app and this deck's controller.
doExitDetect = isfield(sim, 'exit_detect');
if doExitDetect
    ed = sim.exit_detect;
    if ~isfield(ed,'front_m'), ed.front_m = 1.2; end
    if ~isfield(ed,'side_m'),  ed.side_m  = 1.0; end
    if ~isfield(ed,'up_m'),    ed.up_m    = 0.8; end
    if ~isfield(ed,'hold_s'),  ed.hold_s  = 0.6; end
    exitSeenStarted = false;
    exitSeenT0 = 0;
end

% ---- early collision check at t=0 ----
if collision_sphere_obbs(state, sim.robot_radius, env.obbs)
    log.collision = true;
    log.t_collision = 0;
    log.stop_reason = "initial_collision";
    k_end = 1;
    log = trim_log(log, k_end);
    log.t(1) = 0;
    log.x(1) = state.x; log.y(1)=state.y; log.z(1)=state.z; log.yaw(1)=state.yaw;
    log.vx(1)=0; log.vy(1)=0; log.vz(1)=0; log.wz(1)=0;
    for i=1:numel(RCHAN)
        log.(RCHAN{i})(1) = ranges.(RCHAN{i});
    end
    log.rmin(1) = min(cellfun(@(f) ranges.(f), RCHAN));
    log.speed(1)=0;
    [mode, pF] = infer_mode_and_pF9(state, goal, sim, ranges);
    log.mode(1)=mode;
    log.pF(1)=pF;
    log.N = 1;
    log.tlog = 1;
    return;
end

for k = 1:N
    t = (k-1)*dt;
    log.t(k) = t;

    % sensor update (sample-and-hold)
    if t >= t_next_sense - 1e-12
        ranges = raycast_ranges9_obbs(state, env.obbs, env.max_range);
        if isfield(sim, 'sensor_noise')
            ranges = apply_range_noise(ranges, sim.sensor_noise);
        end
        t_next_sense = t_next_sense + sensor_dt;
    end

    % controller
    tic;
    [cmd, mem] = controllerFcn(t, state, ranges, goal, env, sim, mem);
    log.tlog = toc;

    vx = util_clamp(cmd.vx, -vlim(1), vlim(1));
    vy = util_clamp(cmd.vy, -vlim(2), vlim(2));
    vz = util_clamp(cmd.vz, -vlim(3), vlim(3));
    wz = util_clamp(cmd.wz, -wlim,    wlim);

    % log state + cmd + ranges
    log.x(k) = state.x; log.y(k) = state.y; log.z(k) = state.z; log.yaw(k) = state.yaw;
    log.vx(k)=vx; log.vy(k)=vy; log.vz(k)=vz; log.wz(k)=wz;

    for i=1:numel(RCHAN)
        log.(RCHAN{i})(k) = ranges.(RCHAN{i});
    end

    log.rmin(k)  = min(cellfun(@(f) ranges.(f), RCHAN));
    log.speed(k) = sqrt(vx.^2 + vy.^2 + vz.^2);

    [mode, pF] = infer_mode_and_pF9(state, goal, sim, ranges);
    log.mode(k) = mode;
    log.pF(k)   = pF;

    % goal check
    dist = norm([state.x-goal.x, state.y-goal.y, state.z-goal.z]);
    if dist < sim.goal_tol
        log.goal_reached = true;
        log.t_goal = t;
        log.stop_reason = "goal";
        k_end = k;
        break;
    end

    % exit-then-stop check (opt-in, see sim.exit_detect above) -- front
    % is the conservative fan minimum min(rFL,rC,rFR); uses the same
    % (possibly noisy/dropout-held) ranges the controller just saw.
    if doExitDetect
        frontM    = min([ranges.rFL, ranges.rC, ranges.rFR]);
        frontOpen = isfinite(frontM) && (frontM > ed.front_m);
        leftOpen  = isfinite(ranges.rL) && (ranges.rL > ed.side_m);
        rightOpen = isfinite(ranges.rR) && (ranges.rR > ed.side_m);
        upOpen    = isfinite(ranges.rU) && (ranges.rU > ed.up_m);
        openCount = frontOpen + leftOpen + rightOpen + upOpen;

        if openCount >= 3
            if ~exitSeenStarted
                exitSeenStarted = true;
                exitSeenT0 = t;
            elseif (t - exitSeenT0) >= ed.hold_s
                log.exited = true;
                log.t_exited = t;
                log.stop_reason = "exited_tunnel";
                k_end = k;
                break;
            end
        else
            exitSeenStarted = false;
        end
    end

    % integrate (body -> world)
    yaw = state.yaw;
    c = cos(yaw); s = sin(yaw);

    dx   = dt*(vx*c - vy*s);
    dy   = dt*(vx*s + vy*c);
    dz   = dt*vz;
    dyaw = dt*wz;

    newState = state;
    newState.x   = state.x + dx;
    newState.y   = state.y + dy;
    newState.z   = state.z + dz;
    newState.yaw = util_wrapToPi(state.yaw + dyaw);

    % altitude clamp
    newState.z = util_clamp(newState.z, sim.z_limits(1), sim.z_limits(2));

    % collision check
    if collision_sphere_obbs(newState, sim.robot_radius, env.obbs)
        log.collision = true;
        log.t_collision = t;
        log.stop_reason = "collision";
        k_end = k;
        break;
    end

    state = newState;
end

if ~exist('k_end','var')
    k_end = N;
end

log = trim_log(log, k_end);
log.N = k_end;
end

% ---------------- helpers ----------------

function log = trim_log(log, k_end)
fn = fieldnames(log);
for i=1:numel(fn)
    v = log.(fn{i});
    if isnumeric(v) && size(v,1) > k_end
        log.(fn{i}) = v(1:k_end,:);
    elseif isstring(v) && size(v,1) > k_end
        log.(fn{i}) = v(1:k_end,:);
    end
end
end

function [mode, pF] = infer_mode_and_pF9(state, goal, sim, ranges)
% Mirror of infer_mode_and_pF from simulate_controller.m, with rD
% replaced by rFD (see file header note -- no straight-down sensor yet).

if isfield(sim,'is_vertical') && sim.is_vertical
    dz = goal.z - state.z;
    if dz >= 0
        mode = "UP";   pF = ranges.rU;
    else
        mode = "DOWN"; pF = ranges.rFD;
    end
    return;
end

dx = goal.x - state.x; dy = goal.y - state.y; dz = goal.z - state.z;
dxy = hypot(dx,dy);

if dxy < 1e-6
    if dz >= 0
        mode="UP"; pF = ranges.rU;
    else
        mode="DOWN"; pF = ranges.rFD;
    end
    return;
end

if abs(dz) > 1.2*dxy
    if dz >= 0
        mode="UP"; pF = ranges.rU;
    else
        mode="DOWN"; pF = ranges.rFD;
    end
else
    mode="H"; pF = ranges.rC;
end

if ~isfinite(pF), pF = 10.0; end
end
