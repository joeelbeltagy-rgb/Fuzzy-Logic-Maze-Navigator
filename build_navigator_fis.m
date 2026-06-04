% build_navigator_fis.m
% Run this script ONCE to generate navigator.fis
% Strategy: Left-Wall Follower + Obstacle Avoidance

%% ============================================================
%  CREATE MAMDANI FIS
%% ============================================================
fis = mamfis('Name', 'navigator');

%% ============================================================
%  INPUTS  (all sensors: [0, 5] metres)
%% ============================================================

% --- Input 1: Left sensor ---
fis = addInput(fis, [0 5], 'Name', 'Left');
fis = addMF(fis, 'Left', 'trimf', [0.0  0.0  0.6],  'Name', 'TooClose');
fis = addMF(fis, 'Left', 'trimf', [0.4  0.8  1.4],  'Name', 'Close');
fis = addMF(fis, 'Left', 'trimf', [1.0  1.8  2.6],  'Name', 'Safe');
fis = addMF(fis, 'Left', 'trimf', [2.0  5.0  5.0],  'Name', 'Far');

% --- Input 2: Front sensor ---
fis = addInput(fis, [0 5], 'Name', 'Front');
fis = addMF(fis, 'Front', 'trimf', [0.0  0.0  0.6],  'Name', 'TooClose');
fis = addMF(fis, 'Front', 'trimf', [0.4  1.0  1.8],  'Name', 'Close');
fis = addMF(fis, 'Front', 'trimf', [1.4  2.5  3.6],  'Name', 'Safe');
fis = addMF(fis, 'Front', 'trimf', [3.0  5.0  5.0],  'Name', 'Clear');

% --- Input 3: Right sensor ---
fis = addInput(fis, [0 5], 'Name', 'Right');
fis = addMF(fis, 'Right', 'trimf', [0.0  0.0  0.6],  'Name', 'TooClose');
fis = addMF(fis, 'Right', 'trimf', [0.4  0.8  1.4],  'Name', 'Close');
fis = addMF(fis, 'Right', 'trimf', [1.0  1.8  2.6],  'Name', 'Safe');
fis = addMF(fis, 'Right', 'trimf', [2.0  5.0  5.0],  'Name', 'Far');

%% ============================================================
%  OUTPUTS  (wheel speeds: [-2.8, 2.8] m/s)
%% ============================================================

% --- Output 1: vL (Left wheel) ---
fis = addOutput(fis, [-2.8 2.8], 'Name', 'vL');
fis = addMF(fis, 'vL', 'trimf', [-2.8 -2.8 -1.2], 'Name', 'Reverse');
fis = addMF(fis, 'vL', 'trimf', [-1.5  0.0  1.5], 'Name', 'Stop');
fis = addMF(fis, 'vL', 'trimf', [ 0.5  1.4  2.2], 'Name', 'Slow');
fis = addMF(fis, 'vL', 'trimf', [ 1.8  2.8  2.8], 'Name', 'Fast');

% --- Output 2: vR (Right wheel) ---
fis = addOutput(fis, [-2.8 2.8], 'Name', 'vR');
fis = addMF(fis, 'vR', 'trimf', [-2.8 -2.8 -1.2], 'Name', 'Reverse');
fis = addMF(fis, 'vR', 'trimf', [-1.5  0.0  1.5], 'Name', 'Stop');
fis = addMF(fis, 'vR', 'trimf', [ 0.5  1.4  2.2], 'Name', 'Slow');
fis = addMF(fis, 'vR', 'trimf', [ 1.8  2.8  2.8], 'Name', 'Fast');

%% ============================================================
%  RULE BASE
%  Format: [in1 in2 in3, out1 out2, weight, AND=1/OR=2]
%  MF indices match the order added above.
%
%  Left  : 1=TooClose  2=Close  3=Safe  4=Far
%  Front : 1=TooClose  2=Close  3=Safe  4=Clear
%  Right : 1=TooClose  2=Close  3=Safe  4=Far
%  vL/vR : 1=Reverse   2=Stop   3=Slow  4=Fast
%% ============================================================

ruleList = [
% ── EMERGENCY: Front TooClose ─────────────────────────────────
% Spin right in place (vL fast, vR reverse)
  0  1  0   4  1   1  1;   % Front TooClose → hard right spin
  0  2  0   4  2   1  1;   % Front Close    → turn right (vL fast, vR stop)

% ── LEFT-WALL FOLLOWING ───────────────────────────────────────
% Goal: keep Left sensor in "Close" range (~0.6-1.4 m)
% Left TooClose → steer right (reduce vR)
  1  0  0   4  2   1  1;   % Left TooClose → vL Fast, vR Stop  (turn right)
% Left Close (ideal) + front clear → go straight fast
  2  4  0   4  4   1  1;   % Left Close, Front Clear → full speed ahead
  2  3  0   3  4   1  1;   % Left Close, Front Safe  → slight left lean
% Left Safe → nudge left (vR faster than vL)
  3  4  0   3  4   1  1;   % Left Safe, Front Clear  → lean left
  3  3  0   3  4   1  1;   % Left Safe, Front Safe   → lean left
% Left Far → turn hard left (open space on left)
  4  4  0   2  4   1  1;   % Left Far,  Front Clear  → hard left
  4  3  0   2  4   1  1;   % Left Far,  Front Safe   → hard left

% ── OPEN SPACE (all clear) ───────────────────────────────────
  4  4  4   3  4   1  1;   % Everything far → lean left (find wall)

% ── RIGHT WALL PROTECTION ────────────────────────────────────
  0  0  1   3  4   1  1;   % Right TooClose → turn left (vR fast, vL slow)
];

fis = addRule(fis, ruleList);

%% ============================================================
%  SAVE
%% ============================================================
writeFIS(fis, 'navigator');
disp('navigator.fis saved successfully!');

%% ============================================================
%  QUICK SANITY CHECK
%% ============================================================
% Test: open front, left wall close → should go straight fast
out = evalfis(fis, [0.9, 4.0, 2.0]);
fprintf('Sanity check [Left=0.9 Front=4.0 Right=2.0] => vL=%.2f  vR=%.2f\n', out(1), out(2));
