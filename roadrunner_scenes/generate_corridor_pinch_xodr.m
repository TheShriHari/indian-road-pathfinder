function xodrPath = generate_corridor_pinch_xodr(cfg, outDir)
%% GENERATE_CORRIDOR_PINCH_XODR  Write an OpenDRIVE 1.6 file for CorridorPinch.
%
%  Generates a straight rural road with a local lane-width reduction at
%  cfg.pinch_x (simulating an eroded shoulder or parked obstacle).
%  The file is pure XML — no RoadRunner connection needed.
%
%  Inputs:
%    cfg    : struct with fields (see default_cfg below)
%    outDir : directory to write CorridorPinch.xodr into
%
%  Output:
%    xodrPath : absolute path to the written .xodr file
%
%  OpenDRIVE note:
%    Variable lane width is expressed via <laneWidth> polynomial records.
%    Each record is valid from its sOffset to the next record's sOffset.
%    We use three width records:
%      [0 → pinch_x-1]   : nominal width
%      [pinch_x-1 → pinch_x+1]  : reduced width (pinch_reduction applied)
%      [pinch_x+1 → roadLength] : nominal width restored
%    The 2m pinch zone is physically correct for a slow-crossing cattle
%    scenario; the planner must react ≥5m upstream.

if nargin < 2, outDir = fileparts(mfilename('fullpath')); end

% ── Defaults (overridden by cfg fields) ────────────────────────────────────
defaults = struct( ...
    'roadLength',       100.0, ...
    'laneWidth_nominal', 3.7,  ...
    'shoulder_width',    0.3,  ...
    'pinch_x',          45.0, ...
    'pinch_reduction',   0.5, ...  % narrows lane by this much (m) on right side
    'sceneName',        'CorridorPinch' ...
);
if nargin < 1 || isempty(cfg), cfg = defaults; end
fn = fieldnames(defaults);
for k = 1:numel(fn)
    if ~isfield(cfg, fn{k}), cfg.(fn{k}) = defaults.(fn{k}); end
end

% Derived
L   = cfg.roadLength;
lw  = cfg.laneWidth_nominal;
sw  = cfg.shoulder_width;
px  = cfg.pinch_x;
pr  = cfg.pinch_reduction;
lw_pinch = lw - pr;          % reduced lane width at pinch
pinch_start = max(0, px - 1.0);
pinch_end   = min(L, px + 1.0);

% Road UUID (deterministic from scene name)
roadId = '1';
juncId = '-1';

% ── Build XML ─────────────────────────────────────────────────────────────
lines = {};
A = @(s) lines{end+1} = s;  %#ok — builds cell array

A('<?xml version="1.0" encoding="UTF-8"?>');
A('<OpenDRIVE>');
A('  <header revMajor="1" revMinor="6" name="CorridorPinch" version="1.00"');
A(sprintf('          date="%s" north="0" south="0" east="0" west="0" vendor="MATLAB"/>', ...
    datestr(now, 'yyyy-mm-ddTHH:MM:SS')));

% ── Road element ───────────────────────────────────────────────────────────
A(sprintf('  <road name="%s" length="%.4f" id="%s" junction="%s">', ...
    cfg.sceneName, L, roadId, juncId));

%   Link (standalone road — no predecessor/successor)
A('    <link/>');

%   Geometry: single straight segment
A('    <planView>');
A(sprintf('      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="%.4f">', L));
A('        <line/>');
A('      </geometry>');
A('    </planView>');

%   Elevation: flat
A('    <elevationProfile>');
A('      <elevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>');
A('    </elevationProfile>');

%   Lateral profile: flat superelevation
A('    <lateralProfile>');
A('      <superelevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>');
A('    </lateralProfile>');

%   Lane sections — three sections to express variable width
A('    <lanes>');
A(sprintf('      <laneOffset s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>'));

% Helper: write a lane section
%   Lane IDs: -1 = right driving lane, -2 = right shoulder, 1 = left shoulder
%   (OpenDRIVE: negative = right of centreline)
    function emit_lane_section(s_start, lane_w, shld_w)
        A(sprintf('      <laneSection s="%.4f">', s_start));
        A('        <left>');
        A('          <lane id="1" type="shoulder" level="false">');
        A(sprintf('            <width sOffset="0.0" a="%.4f" b="0.0" c="0.0" d="0.0"/>', shld_w));
        A('            <roadMark sOffset="0.0" type="none" weight="standard" color="standard" laneChange="none"/>');
        A('          </lane>');
        A('        </left>');
        A('        <center>');
        A('          <lane id="0" type="none" level="false">');
        A('            <roadMark sOffset="0.0" type="broken" weight="standard" color="white" laneChange="both"/>');
        A('          </lane>');
        A('        </center>');
        A('        <right>');
        A('          <lane id="-1" type="driving" level="false">');
        A(sprintf('            <width sOffset="0.0" a="%.4f" b="0.0" c="0.0" d="0.0"/>', lane_w));
        A('            <roadMark sOffset="0.0" type="none" weight="standard" color="standard" laneChange="none"/>');
        A('          </lane>');
        A('          <lane id="-2" type="shoulder" level="false">');
        A(sprintf('            <width sOffset="0.0" a="%.4f" b="0.0" c="0.0" d="0.0"/>', shld_w));
        A('            <roadMark sOffset="0.0" type="none" weight="standard" color="standard" laneChange="none"/>');
        A('          </lane>');
        A('        </right>');
        A('      </laneSection>');
    end

% Section 1: nominal width from 0 to pinch_start
emit_lane_section(0.0,          lw,       sw);
% Section 2: pinched width from pinch_start to pinch_end
emit_lane_section(pinch_start,  lw_pinch, sw);
% Section 3: restored nominal from pinch_end to end
emit_lane_section(pinch_end,    lw,       sw);

A('    </lanes>');

%   Objects & signals (none for this scene)
A('    <objects/>');
A('    <signals/>');
A('  </road>');

% ── GeoReference (flat ENU, no projection) ────────────────────────────────
A('  <geoReference><![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m]]></geoReference>');
A('</OpenDRIVE>');

% ── Write file ────────────────────────────────────────────────────────────
xodrPath = fullfile(outDir, [cfg.sceneName '.xodr']);
fid = fopen(xodrPath, 'w', 'n', 'UTF-8');
if fid < 0
    error('generate_corridor_pinch_xodr: cannot open "%s" for writing', xodrPath);
end
for k = 1:numel(lines)
    fprintf(fid, '%s\n', lines{k});
end
fclose(fid);

fprintf('[xodr] Written: %s\n', xodrPath);
fprintf('       Road length: %.1f m | Nominal lane: %.1f m | Pinch @x=%.1f: %.1f m\n', ...
    L, lw, px, lw_pinch);
end
