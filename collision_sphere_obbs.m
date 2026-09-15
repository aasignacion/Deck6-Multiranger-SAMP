function tf = collision_sphere_obbs(state_or_xyz, radius, obbs)
%COLLISION_SPHERE_OBBS  Sphere-vs-OBB collision.
% Signature: tf = collision_sphere_obbs(p_or_state, radius, obbs)
% p_or_state: struct with fields x,y,z OR numeric [x y z]
% radius: sphere radius
% obbs: struct array. Supports either:
%   (A) fields: c (3x1), R (3x3), h (3x1 half-size)
%   (B) fields: T (4x4), size (1x3 full size)

% --- unify position ---
if isstruct(state_or_xyz)
    p = [state_or_xyz.x; state_or_xyz.y; state_or_xyz.z];
else
    p = state_or_xyz(:);
    if numel(p) ~= 3
        error('collision_sphere_obbs: p must be 3 elements.');
    end
end

tf = false;
if isempty(obbs), return; end

for i = 1:numel(obbs)
    % --- read OBB in either format ---
    if isfield(obbs, 'c') && isfield(obbs, 'R') && isfield(obbs, 'h')
        c = obbs(i).c(:);
        R = obbs(i).R;
        h = obbs(i).h(:);
    elseif isfield(obbs, 'T') && isfield(obbs, 'size')
        T = obbs(i).T;
        c = T(1:3,4);
        R = T(1:3,1:3);
        h = 0.5 * obbs(i).size(:);
    else
        error('collision_sphere_obbs: Unsupported OBB format. Expected fields {c,R,h} or {T,size}.');
    end

    % --- closest point on OBB to sphere center ---
    % transform to box local coordinates: q = R'*(p - c)
    q = R'*(p - c);

    % clamp to box extents
    qc = min(max(q, -h), h);

    % back to world, distance to closest point
    p_closest = c + R*qc;
    d2 = sum((p - p_closest).^2);

    if d2 <= radius^2
        tf = true;
        return;
    end
end
end