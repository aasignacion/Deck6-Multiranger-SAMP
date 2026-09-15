
function obbs = load_sdf_obbs(sdfPath)
%LOAD_SDF_OBBS  Parse an SDF world and return oriented bounding boxes (OBBs)
% for all <collision><geometry><box> elements.
%
% obbs(i) has fields:
%   c  (3x1) center in world
%   R  (3x3) rotation matrix of box frame in world
%   h  (3x1) half-sizes [hx;hy;hz]
%   name (string) collision name (optional)
%
% Notes:
% - Applies hierarchical poses: model + link + collision.
% - Supports arbitrary RPY in poses (not just zero).
% - Ignores planes (ground) by design for raycasting safety; add if needed.

assert(isfile(sdfPath), 'File not found: %s', sdfPath);

try
    doc = xmlread(sdfPath);
catch ME
    error('Failed to read SDF: %s', ME.message);
end

models = doc.getElementsByTagName('model');
obbs = struct('c',{},'R',{},'h',{},'name',{});

for mi = 0:models.getLength-1
    modelNode = models.item(mi);
    modelPose = parsePoseText(getChildText(modelNode,'pose'));
    Tm = rpyPoseToT(modelPose);

    links = childrenByTag(modelNode,'link');
    for li = 1:numel(links)
        link = links{li};
        linkPose = parsePoseText(getChildText(link,'pose'));
        Tl = rpyPoseToT(linkPose);

        colls = childrenByTag(link,'collision');
        for ci = 1:numel(colls)
            coll = colls{ci};
            cName = char(getAttr(coll,'name'));

            collPose = parsePoseText(getChildText(coll,'pose'));
            Tc = rpyPoseToT(collPose);

            geom = firstChildByTag(coll,'geometry');
            if isempty(geom), continue; end
            box = firstChildByTag(geom,'box');
            if isempty(box), continue; end

            sz = parseNumbers(getChildText(box,'size'),3);
            if any(~isfinite(sz)) || any(sz<=0), continue; end

            Tw = Tm * Tl * Tc;
            Rw = Tw(1:3,1:3);
            cw = Tw(1:3,4);

            o.c = cw;
            o.R = Rw;
            o.h = (sz(:) / 2);
            o.name = string(cName);
            obbs(end+1) = o; %#ok<AGROW>
        end
    end
end

end

% ===================== DOM utilities (from your visualizer style) =====================
function nodes = childrenByTag(node, tag)
nodes = {};
kids = node.getChildNodes;
for i = 0:kids.getLength-1
    n = kids.item(i);
    if n.getNodeType()==n.ELEMENT_NODE && strcmpi(char(n.getNodeName), tag)
        nodes{end+1} = n; %#ok<AGROW>
    end
end
end

function n = firstChildByTag(node, tag)
nodes = childrenByTag(node, tag);
if isempty(nodes), n = []; else, n = nodes{1}; end
end

function txt = getChildText(node, tag)
txt = '';
n = firstChildByTag(node, tag);
if isempty(n), return; end
try
    txt = char(n.getTextContent());
catch
    t = ''; c = n.getFirstChild;
    while ~isempty(c)
        if c.getNodeType()==c.TEXT_NODE || c.getNodeType()==c.CDATA_SECTION_NODE
            t = [t, char(c.getNodeValue)]; %#ok<AGROW>
        end
        c = c.getNextSibling;
    end
    txt = t;
end
txt = strtrim(txt);
end

function val = getAttr(node, name)
val = '';
attrs = node.getAttributes;
if isempty(attrs), return; end
nd = attrs.getNamedItem(name);
if ~isempty(nd), val = char(nd.getNodeValue); end
end

% ===================== math helpers =====================
function p = parsePoseText(s)
p = parseNumbers(s,6);
if any(~isfinite(p)), p = [0 0 0 0 0 0]; end
end

function nums = parseNumbers(str, n)
if nargin<2, n = []; end
if isempty(str), nums = nan(1,max(1,n)); return; end
s = strtrim(str);
s = strrep(s, char(160),' ');
s = strrep(s, char(12288),' ');
arr = sscanf(s,'%f');
if isempty(n), nums = arr(:)'; else
    nums = nan(1,n); m = min(numel(arr),n);
    nums(1:m) = arr(1:m);
end
end

function T = rpyPoseToT(p)
x=p(1); y=p(2); z=p(3); r=p(4); pt=p(5); yw=p(6);
Rx=[1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
Ry=[cos(pt) 0 sin(pt); 0 1 0; -sin(pt) 0 cos(pt)];
Rz=[cos(yw) -sin(yw) 0; sin(yw) cos(yw) 0; 0 0 1];
R = Rz*Ry*Rx;
T = eye(4); T(1:3,1:3)=R; T(1:3,4)=[x;y;z];
end
