function ranges = raycast_ranges9_obbs(state, obbs, maxRange)
%RAYCAST_RANGES9_OBBS  Simulate the 9-sensor vl53l8cx deck via ray-OBB intersection.
%
% Same ray-casting engine as raycast_ranges_obbs.m (multiranger, 6 rays),
% extended to the vl53l8cx deck's 9-sensor layout: a 5-beam front fan
% (left/center/right/up/down) plus single left/right/back/up beams.
%
% state: struct('x','y','z','yaw')
% obbs: struct array from load_sdf_obbs
% maxRange: max sensor range (m)
%
% Returns struct with fields rFL,rC,rFR,rFU,rFD,rL,rR,rB,rU (all in
% meters, capped at maxRange), matching the physical sensor mapping the
% user confirmed for the current 9-sensor build:
%   0 front-left, 1 center, 2 front-right, 3 front-up, 4 front-down,
%   5 left, 6 right, 7 back, 8 up.
%
% ASSUMPTION -- ray angles below (+-45 deg for the front fan's left/
% right beams, +-30 deg elevation for front-up/front-down) are carried
% over from the documented mounting angles of the original 11-sensor
% prototype board (docs/vl53l8cx-position.md in the firmware repo),
% since that's the only calibration data available for this deck. If
% you have actual measured angles for the 9-sensor build, update
% ANG_FAN_DEG / ANG_TILT_DEG below -- nothing else in this file depends
% on the specific values.

ANG_FAN_DEG  = 45;   % front-left / front-right yaw offset from center
ANG_TILT_DEG = 30;   % front-up / front-down pitch offset from center

x = state.x; y = state.y; z = state.z; yaw = state.yaw;
o = [x;y;z];

fanRad  = deg2rad(ANG_FAN_DEG);
tiltRad = deg2rad(ANG_TILT_DEG);

% Body-frame unit directions (CF convention: +x forward, +y left, +z up)
dirs_body = struct( ...
    'FL',[cos(fanRad);  sin(fanRad); 0], ...
    'C', [1;0;0], ...
    'FR',[cos(fanRad); -sin(fanRad); 0], ...
    'FU',[cos(tiltRad); 0;  sin(tiltRad)], ...
    'FD',[cos(tiltRad); 0; -sin(tiltRad)], ...
    'L', [0;1;0], ...
    'R', [0;-1;0], ...
    'B', [-1;0;0], ...
    'U', [0;0;1]);

Rz = [cos(yaw) -sin(yaw) 0;
      sin(yaw)  cos(yaw) 0;
      0         0        1];

ranges.rFL = castOne(o, Rz*dirs_body.FL, obbs, maxRange);
ranges.rC  = castOne(o, Rz*dirs_body.C,  obbs, maxRange);
ranges.rFR = castOne(o, Rz*dirs_body.FR, obbs, maxRange);
ranges.rFU = castOne(o, Rz*dirs_body.FU, obbs, maxRange);
ranges.rFD = castOne(o, Rz*dirs_body.FD, obbs, maxRange);
ranges.rL  = castOne(o, Rz*dirs_body.L,  obbs, maxRange);
ranges.rR  = castOne(o, Rz*dirs_body.R,  obbs, maxRange);
ranges.rB  = castOne(o, Rz*dirs_body.B,  obbs, maxRange);
ranges.rU  = castOne(o, Rz*dirs_body.U,  obbs, maxRange);

end

function r = castOne(o, d, obbs, maxRange)
% Return distance to closest intersection, capped at maxRange.
r = maxRange;
for i = 1:numel(obbs)
    [hit, t] = ray_obb(o, d, obbs(i).c, obbs(i).R, obbs(i).h);
    if hit && t > 0 && t < r
        r = t;
    end
end
end

function [hit, tmin] = ray_obb(o, d, c, R, h)
% Ray (o + t d) vs OBB (center c, rotation R, half-size h)
% Using slab method in box-local coordinates.
% Returns hit and nearest positive t.

% transform ray into box local coordinates
ol = R'*(o - c);
dl = R'*d;

t1 = (-h - ol) ./ (dl + eps);
t2 = ( h - ol) ./ (dl + eps);

tminv = min(t1, t2);
tmaxv = max(t1, t2);

tmin = max([tminv(1), tminv(2), tminv(3)]);
tmax = min([tmaxv(1), tmaxv(2), tmaxv(3)]);

hit = (tmax >= max(tmin, 0));
if ~hit
    tmin = inf;
    return;
end

% nearest positive intersection
if tmin < 0
    tmin = tmax; % we're inside; exit intersection
end
end
