% add_freq_scope.m
% Add a frequency-domain-free, TIME-domain scope showing the commanded drive
% frequency in Hz, branched off the signal that already feeds the UDP Send block.
%
% Why: the signal sent to Python is the raw controller output (~1.169 - 1.837).
% The vision node multiplies it by 50 to get Hz. The model's existing Scope is
% wired to the Simscape mechanical plant, not to this signal, so nothing in
% Simulink shows the commanded frequency over time. This script adds:
%
%       <source of UDP Send> --branch--> [Gain x50] --> [Frequency Scope]
%
% No existing block, line, or parameter is modified. The added Gain reproduces
% the exact conversion already performed in vision/probe_detection.py
% (FREQ_GAIN = 50.0), so the scope reads in the same units as the README.
%
% Run it AFTER run_controller, with the repo copy of the model open:
%
%   >> add_freq_scope
%
% Safe to run more than once: it removes its own previously added blocks first.
%
% Author: Ting Lung, NCHU BIME.

model    = 'dampedsystem';
thisDir  = fileparts(mfilename('fullpath'));
gainName = [model '/Hz Gain x50'];
scopeNm  = [model '/Frequency Scope'];

% --- 0. Safety: refuse to edit a copy of the model outside this folder -------
if ~bdIsLoaded(model)
    error('add_freq_scope:notLoaded', ...
          'Model %s is not open. Run run_controller first.', model);
end

loadedPath   = get_param(model, 'FileName');
expectedPath = fullfile(thisDir, [model '.slx']);
if ~strcmpi(loadedPath, expectedPath)
    error('add_freq_scope:wrongCopy', ...
        ['The open model is NOT the repo copy, so changes would not reach the repo.\n' ...
         '  open    : %s\n  expected: %s\n' ...
         'Run these first:\n' ...
         '    bdclose all\n' ...
         '    cd ''%s''\n    run_controller'], loadedPath, expectedPath, thisDir);
end

% --- 1. Stop the simulation if it is running --------------------------------
if ~strcmp(get_param(model, 'SimulationStatus'), 'stopped')
    fprintf('Stopping the running simulation...\n');
    set_param(model, 'SimulationCommand', 'stop');
    pause(1);
end

% --- 2. Remove anything this script added on a previous run ------------------
for name = {gainName, scopeNm}
    if ~isempty(find_system(model, 'SearchDepth', 1, 'LookUnderMasks', 'all', ...
                            'Name', extractAfter(name{1}, [model '/'])))
        delete_block(name{1});
    end
end

% --- 3. Locate the signal that feeds UDP Send -------------------------------
udpBlock = [model '/UDP Send'];
ph       = get_param(udpBlock, 'PortHandles');
ln       = get_param(ph.Inport(1), 'Line');

if ln <= 0
    error('add_freq_scope:noLine', 'UDP Send has no incoming signal line.');
end

srcPort = get_param(ln, 'SrcPortHandle');
srcName = get_param(get_param(ln, 'SrcBlockHandle'), 'Name');
fprintf('Branching from: %s (port %d)\n', ...
        strrep(srcName, newline, ' '), get_param(srcPort, 'PortNumber'));

% --- 4. Add the gain and the scope, placed below UDP Send -------------------
p = get_param(udpBlock, 'Position');
add_block('simulink/Math Operations/Gain', gainName, ...
          'Gain', '50', 'Position', [p(1), p(4)+70, p(1)+40, p(4)+110]);
add_block('simulink/Sinks/Scope', scopeNm, ...
          'Position', [p(1)+110, p(4)+70, p(1)+150, p(4)+110]);

gp = get_param(gainName, 'PortHandles');
sp = get_param(scopeNm,  'PortHandles');
add_line(model, srcPort,       gp.Inport(1),  'autorouting', 'on');
add_line(model, gp.Outport(1), sp.Inport(1),  'autorouting', 'on');

% --- 5. Fix the scope axes so the trace does not autoscale while recording ---
% An autoscaling Y axis makes the frequency step look like noise instead of a
% control action. 0-150 Hz is the clamp range enforced in probe_detection.py.
axesSet = false;

% Works on R2025b and other recent releases.
try
    set_param(scopeNm, 'YMin', '0', 'YMax', '150', 'TimeSpan', '15');
    axesSet = true;
catch
end

% Older releases exposed the same settings through a configuration object.
if ~axesSet
    try
        cfg = get_param(scopeNm, 'ScopeConfiguration');
        cfg.YLimits  = [0 150];
        cfg.YLabel   = 'Commanded frequency (Hz)';
        cfg.TimeSpan = 15;
        axesSet = true;
    catch
    end
end

if axesSet
    fprintf('Scope axes fixed at 0-150 Hz over a 15 s window.\n');
else
    fprintf(['\n[note] Could not set the scope axes from code on this MATLAB\n' ...
             '       version. Set them by hand instead:\n' ...
             '       Scope > View > Configuration Properties > Display\n' ...
             '       Y-limits 0 and 150, Time span 15\n']);
end

% --- 6. Save -----------------------------------------------------------------
save_system(model);

fprintf('\nDone. Added and saved into:\n  %s\n', loadedPath);
fprintf(['Open the new scope and press Run:\n' ...
         '    open_system(''%s'')\n\n' ...
         'Expected trace: ~58.4 Hz for the dispersed frame,\n' ...
         '                ~91.8 Hz for the piled frame.\n'], scopeNm);
