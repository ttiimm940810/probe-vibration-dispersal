% run_controller.m
% Entry point for the control layer of the closed-loop probe dispersal system.
%
% Loads the Mamdani Type-1 fuzzy inference system, prints its structure so the
% 2 inputs / 2 outputs / 9 rules can be verified, and opens the Simulink model.
%
% Run this BEFORE starting the Python vision node, so that the UDP Receive
% block is already listening on port 12345 when the first packet arrives.
%
%   >> cd control
%   >> run_controller
%
% Then in Simulink: set stop time to inf, press Run, and open the Scope.
%
% UDP configuration expected by vision/probe_detection.py:
%   UDP Receive : local port 12345, 16-byte payload -> two doubles [U, R]
%   UDP Send    : 127.0.0.1, remote port 12346, one double (controller output)
%
% Author: Ting Lung, NCHU BIME.

thisDir   = fileparts(mfilename('fullpath'));
modelName = 'dampedsystem';
fisFile   = fullfile(thisDir, 'mamdanitype1.fis');

% --- 1. Load the fuzzy inference system -------------------------------------
fuzzy_brain = readfis(fisFile);   %#ok<NASGU>
fprintf('Loaded FIS: %s\n', fisFile);
fprintf('  inputs : %d\n', numel(fuzzy_brain.Inputs));
fprintf('  outputs: %d\n', numel(fuzzy_brain.Outputs));
fprintf('  rules  : %d\n', numel(fuzzy_brain.Rules));
assignin('base', 'fuzzy_brain', fuzzy_brain);

% --- 2. Open the Simulink model ---------------------------------------------
% Guard against a copy of dampedsystem.slx from another folder already being
% loaded (a stale copy elsewhere on the MATLAB path shadows this one, and you
% end up editing the wrong file). If a different copy is open it is closed
% first -- UNSAVED CHANGES IN THAT COPY ARE DISCARDED.
addpath(thisDir);
modelPath = fullfile(thisDir, [modelName '.slx']);

if bdIsLoaded(modelName)
    loadedPath = get_param(modelName, 'FileName');
    if ~strcmpi(loadedPath, modelPath)
        fprintf('A different copy of %s was open; closing it:\n  %s\n', ...
                modelName, loadedPath);
        bdclose(modelName);
    end
end

if bdIsLoaded(modelName)
    open_system(modelName);
else
    open_system(modelPath);
end

fprintf('Model in use: %s\n', get_param(modelName, 'FileName'));

% --- 3. Demo-friendly simulation settings (idempotent) ----------------------
% Pace the simulation to wall-clock time. Without this Simulink runs the
% model many times faster than real time, so a scope with a 30 s time span
% shows well under a second of real time and every transition is compressed
% into a vertical line.
set_param(modelName, 'StopTime', 'inf', 'EnablePacing', 'on', 'PacingRate', 1);

% Frequency Scope: fixed 0-150 Hz axis, 30 s window, scroll (not wrap) so a
% step stays on screen and moves left instead of being erased at the window
% boundary. Wrapped in try/catch so an older model without the scope still
% opens.
try
    set_param([modelName '/Frequency Scope'], ...
              'YMin', '0', 'YMax', '150', ...
              'TimeSpan', '30', 'TimeSpanOverrunAction', 'Scroll');
catch
    fprintf('[note] Frequency Scope not found - run add_freq_scope to add it.\n');
end

% The Spectrum Analyzer is not part of the demo; stop it from popping up on
% every Run so only the Frequency Scope opens.
try
    set_param([modelName '/Spectrum Analyzer'], 'OpenAtSimulationStart', 'off');
catch
end

fprintf(['\nModel open (stop time inf, real-time pacing on).\n' ...
         'Checklist before pressing Run:\n' ...
         '  1. UDP Receive local port  = 12345\n' ...
         '  2. UDP Send  remote port   = 12346 (127.0.0.1)\n' ...
         '  3. open_system(''%s/Frequency Scope'')\n' ...
         '\nThen start the vision node:\n' ...
         '  python vision/probe_detection.py --source assets/dispersing.mp4\n\n'], modelName);
