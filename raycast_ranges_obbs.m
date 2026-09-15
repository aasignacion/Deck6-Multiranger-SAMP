
function ranges = raycast_ranges_obbs(state, obbs, maxRange)
%RAYCAST_RANGES_OBBS  Simulate multiranger-style ranges via ray-OBB intersection.
%
% state: struct('x','y','z','yaw')
% obbs: struct array from load_sdf_obbs
% maxRange: max sensor range (m)
%
% Returns struct with fields rF,rB,rL,rR,rU,rD.

x = state.x; y = state.y; z = state.z; yaw = state.yaw;
o = [x;y;z];

% Body-frame unit directions (CF convention: +x forward, +y left, +z up)
dirs_body = struct( ...
    'F',[1;0;0], ...
    'B',[-1;0;0], ...
    'L',[0;1;0], ...
    'R',[0;-1;0], ...
    'U',[0;0;1], ...
    'D',[0;0;-1]);

Rz = [cos(yaw) -sin(yaw) 0;
      sin(yaw)  cos(yaw) 0;
      0         0        1];

ranges.rF = castOne(o, Rz*dirs_body.F, obbs, maxRange);
ranges.rB = castOne(o, Rz*dirs_body.B, obbs, maxRange);
ranges.rL = castOne(o, Rz*dirs_body.L, obbs, maxRange);
ranges.rR = castOne(o, Rz*dirs_body.R, obbs, maxRange);
ranges.rU = castOne(o, Rz*dirs_body.U, obbs, maxRange);
ranges.rD = castOne(o, Rz*dirs_body.D, obbs, maxRange);

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
