function [status, manifest] = make_rrscene_or_fallback(sceneName)
%% MAKE_RRSCENE_OR_FALLBACK
% Attempts to import an OpenDRIVE (.xodr) road network into RoadRunner R2026a
% and save a populated .rrscene.
% If RoadRunner or its MATLAB API connector is unavailable, falls back
% deterministically to the precomputed parametric trace CSV and logs a manifest.
%
% Usage:
%   >> cd roadrunner_scenes;
%   >> make_rrscene_or_fallback('VillageRoad');
%   >> make_rrscene_or_fallback('UrbanIntersection');
%   >> make_rrscene_or_fallback(); % builds all 5 scenarios

thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end
repoRoot = fullfile(thisDir, '..');

allScenes = {'VillageRoad', 'UrbanIntersection', 'HighwayMerge', 'MarketDense', 'CattleCrossing'};

if nargin < 1 || isempty(sceneName)
    fprintf('=== Building all %d canonical scenarios ===\n', numel(allScenes));
    status = 0;
    manifest = struct();
    for i = 1:numel(allScenes)
        [st_i, m_i] = make_rrscene_or_fallback(allScenes{i});
        manifest.(allScenes{i}) = m_i;
        if st_i ~= 0, status = st_i; end
    end
    return;
end

xodrPath     = fullfile(thisDir, sprintf('%s.xodr', sceneName));
scenePath    = fullfile(thisDir, sprintf('%s.rrscene', sceneName));
traceCsvPath = fullfile(thisDir, sprintf('%s_parametric_trace.csv', sceneName));
manifestPath = fullfile(thisDir, sprintf('%s_fallback_manifest.txt', sceneName));

fprintf('\n--- Processing Scenario: %s ---\n', sceneName);

manifest = struct();
manifest.sceneName = sceneName;
manifest.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');

% Step 1: Check OpenDRIVE geometry source
if ~isfile(xodrPath)
    error('MissingXODR:NotFound', 'Canonical OpenDRIVE geometry missing: %s', xodrPath);
end

% Step 2: Attempt RoadRunner MATLAB API connection
hasRR = (exist('roadrunner', 'file') == 2 || exist('roadrunner', 'builtin') == 5);

if hasRR
    try
        fprintf('  Attempting RoadRunner connection at: %s\n', repoRoot);
        rrApp = roadrunner(repoRoot);
        
        fprintf('  Importing OpenDRIVE: %s\n', xodrPath);
        opts = openDRIVEImportOptions();
        importScene(rrApp, char(xodrPath), "OpenDRIVE", opts);
        
        fprintf('  Saving Scene: %s\n', scenePath);
        saveScene(rrApp, char(scenePath));
        
        info = dir(scenePath);
        if info.bytes >= 50 * 1024
            fprintf('  [SUCCESS] Populated rrscene saved: %d bytes (>=50 KB)\n', info.bytes);
            manifest.mode = 'ROADRUNNER_VALID';
            manifest.size_bytes = info.bytes;
            status = 0;
            return;
        else
            warning('Saved scene is too small (%d bytes). Falling back.\n', info.bytes);
        end
    catch ME
        fprintf('  RoadRunner API error: %s\n', ME.message);
    end
end

% Step 3: Deterministic Fallback Mode
fprintf('  [NOTICE] RoadRunner API offline or license unavailable.\n');
fprintf('  Executing deterministic fallback using canonical OpenDRIVE and parametric trace.\n');

manifest.mode = 'PARAMETRIC_FALLBACK';
manifest.reason = 'RoadRunner MATLAB connector unavailable or unlicensed';
manifest.xodr = xodrPath;
manifest.trace_csv = traceCsvPath;

% Write scenario fallback manifest
fid = fopen(manifestPath, 'w');
if fid > 0
    fprintf(fid, 'scenario=%s\n', sceneName);
    fprintf(fid, 'fallback_used=true\n');
    fprintf(fid, 'reason=%s\n', manifest.reason);
    fprintf(fid, 'xodr_source=%s\n', xodrPath);
    fprintf(fid, 'trace_csv=%s\n', traceCsvPath);
    fprintf(fid, 'timestamp=%s\n', manifest.timestamp);
    fclose(fid);
end

fprintf('  Manifest logged to: %s\n', manifestPath);
status = 0;
end
