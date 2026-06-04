# Fuzzy Logic Maze Controller - Detailed Report Source

## 1. Project Summary

This project implements an autonomous fuzzy-logic controller for a differential-drive robot in a randomly generated MATLAB maze simulator. The robot starts near the bottom-left of the maze and must reach the goal near the top-right without hitting walls.

The final submitted controller is based on a Mamdani fuzzy inference system stored in `navigator.fis`, combined with a small amount of stateful MATLAB logic in `myController.m`. The FIS provides the base wall-following and obstacle-avoidance behavior, while the MATLAB controller adds practical navigation memory, smoother turns, launch stabilization, goal-aware turn selection, speed scaling, and stuck recovery.

The main goal during development was not only to reach the maze goal, but to do it reliably, smoothly, and quickly. Several versions were tested. The version saved in this folder is the best stable version found during tuning.

## 2. Submission Folder Contents

This folder, `final best stable`, contains everything needed to run, inspect, and document the final solution.

| File | Purpose |
|---|---|
| `myController.m` | Final robot controller. This is the main file submitted as the robot brain. |
| `navigator.fis` | Final saved Mamdani fuzzy inference system used by the controller. |
| `build_navigator_fis.m` | MATLAB script that programmatically rebuilds `navigator.fis`. |
| `runSimulation.m` | Convenience script that runs the simulator using `myController`. |
| `mazeSim.m` | Provided/edited simulator environment with UI, sensors, robot physics, and maze generation. |
| `controller_fis_explanation.md` | Earlier explanation draft. |
| `DETAILED_REPORT_SOURCE.md` | This detailed report source file. |
| `Fuzzy_Logic_Maze_Challenge.pdf` | Original assignment brief. |
| `maze.png` | UI icon used by the simulator. |
| `MATLABMazeRunnerWorking.prj` | MATLAB project file. |
| `input_left_mf.png` | Generated plot of Left input membership functions. |
| `input_front_mf.png` | Generated plot of Front input membership functions. |
| `input_right_mf.png` | Generated plot of Right input membership functions. |
| `output_vL_mf.png` | Generated plot of left-wheel output membership functions. |
| `output_vR_mf.png` | Generated plot of right-wheel output membership functions. |
| `surface_left_front_vL.png` | Generated FIS surface plot for Left/Front inputs to `vL`. |
| `surface_left_front_vR.png` | Generated FIS surface plot for Left/Front inputs to `vR`. |

## 3. Original Challenge Requirements

The assignment requires designing a MATLAB Fuzzy Logic Inference System for an autonomous differential-drive robot.

The important constraints are:

| Requirement | Final implementation |
|---|---|
| Exactly 3 FIS inputs | `Left`, `Front`, `Right` sensor distances |
| Input range | `[0, 5]` meters |
| Exactly 2 FIS outputs | `vL`, `vR` wheel speeds |
| Output range | `[-2.8, 2.8]` m/s |
| FIS saved as `.fis` file | `navigator.fis` |
| Controller evaluates FIS from MATLAB | `myController.m` uses `readfis` and `evalfis` |
| Avoid reloading FIS every frame | `persistent fis` is used |
| Stop at goal | `state.goalReached` sets both wheel speeds to zero |
| Avoid walls | Side/front sensor logic and turn modes protect against collisions |
| Deliver report screenshots | Membership function and surface plots are included as PNG files |

## 4. High-Level Final Strategy

The final controller is a hybrid of fuzzy logic and lightweight state logic.

The FIS itself implements reactive wall following and obstacle avoidance. It receives the three distance sensors and outputs two wheel-speed suggestions. The MATLAB controller then uses those outputs as part of a larger navigation strategy.

The main final behaviors are:

1. Drive fast in open corridors.
2. Stay centered between walls.
3. Slow down only when needed near a wall or during a turn.
4. Start cleanly from the first cell without early wobble.
5. Begin turns before collision, but not so early that turns become too wide.
6. Commit briefly after a turn so the robot does not hesitate or oscillate.
7. Use the robot's `(x, y)` position to prefer turns that point toward the top-right goal.
8. If the robot is nose-to-wall, reverse slightly while turning out instead of staying stuck.

## 5. Fuzzy Inference System Overview

The FIS is named `navigator`.

| Property | Value |
|---|---|
| FIS type | Mamdani |
| Number of inputs | 3 |
| Number of outputs | 2 |
| Number of rules | 11 |
| AND method | `min` |
| OR method | `max` |
| Implication method | `min` |
| Aggregation method | `max` |
| Defuzzification method | `centroid` |

The FIS is stored in `navigator.fis`. It can also be regenerated using `build_navigator_fis.m`.

## 6. FIS Inputs

### 6.1 Input 1: Left

The `Left` input represents the distance from the robot to the wall on its left side, in meters.

Range: `[0, 5]`

| MF index | Name | Type | Parameters | Meaning |
|---:|---|---|---|---|
| 1 | `TooClose` | triangular | `[0, 0, 0.6]` | Left wall is dangerously close. |
| 2 | `Close` | triangular | `[0.4, 0.8, 1.4]` | Left wall is near/ideal for wall following. |
| 3 | `Safe` | triangular | `[1.0, 1.8, 2.6]` | Left wall is visible but not close. |
| 4 | `Far` | triangular | `[2.0, 5.0, 5.0]` | Left side is open. |

Generated figure: `input_left_mf.png`

### 6.2 Input 2: Front

The `Front` input represents the distance from the robot to the wall in front of it.

Range: `[0, 5]`

| MF index | Name | Type | Parameters | Meaning |
|---:|---|---|---|---|
| 1 | `TooClose` | triangular | `[0, 0, 0.6]` | Front wall is dangerously close. |
| 2 | `Close` | triangular | `[0.4, 1.0, 1.8]` | Front wall is approaching. |
| 3 | `Safe` | triangular | `[1.4, 2.5, 3.6]` | Forward path has usable space. |
| 4 | `Clear` | triangular | `[3.0, 5.0, 5.0]` | Forward path is very open. |

Generated figure: `input_front_mf.png`

### 6.3 Input 3: Right

The `Right` input represents the distance from the robot to the wall on its right side.

Range: `[0, 5]`

| MF index | Name | Type | Parameters | Meaning |
|---:|---|---|---|---|
| 1 | `TooClose` | triangular | `[0, 0, 0.6]` | Right wall is dangerously close. |
| 2 | `Close` | triangular | `[0.4, 0.8, 1.4]` | Right wall is near. |
| 3 | `Safe` | triangular | `[1.0, 1.8, 2.6]` | Right wall is visible but not close. |
| 4 | `Far` | triangular | `[2.0, 5.0, 5.0]` | Right side is open. |

Generated figure: `input_right_mf.png`

## 7. FIS Outputs

### 7.1 Output 1: vL

`vL` is the left wheel speed in meters per second.

Range: `[-2.8, 2.8]`

| MF index | Name | Type | Parameters | Meaning |
|---:|---|---|---|---|
| 1 | `Reverse` | triangular | `[-2.8, -2.8, -1.2]` | Left wheel reverses. |
| 2 | `Stop` | triangular | `[-1.5, 0, 1.5]` | Left wheel near stopped/neutral. |
| 3 | `Slow` | triangular | `[0.5, 1.4, 2.2]` | Left wheel moves forward slowly. |
| 4 | `Fast` | triangular | `[1.8, 2.8, 2.8]` | Left wheel moves forward quickly. |

Generated figure: `output_vL_mf.png`

### 7.2 Output 2: vR

`vR` is the right wheel speed in meters per second.

Range: `[-2.8, 2.8]`

| MF index | Name | Type | Parameters | Meaning |
|---:|---|---|---|---|
| 1 | `Reverse` | triangular | `[-2.8, -2.8, -1.2]` | Right wheel reverses. |
| 2 | `Stop` | triangular | `[-1.5, 0, 1.5]` | Right wheel near stopped/neutral. |
| 3 | `Slow` | triangular | `[0.5, 1.4, 2.2]` | Right wheel moves forward slowly. |
| 4 | `Fast` | triangular | `[1.8, 2.8, 2.8]` | Right wheel moves forward quickly. |

Generated figure: `output_vR_mf.png`

## 8. FIS Rule Base

The FIS contains 11 rules. In the `.fis` file, `0` means "do not care" for that input.

| Rule | Left | Front | Right | vL | vR | Intended behavior |
|---:|---|---|---|---|---|---|
| 1 | Any | `TooClose` | Any | `Fast` | `Reverse` | Spin right when the front wall is dangerously close. |
| 2 | Any | `Close` | Any | `Fast` | `Stop` | Turn right as the front wall approaches. |
| 3 | `TooClose` | Any | Any | `Fast` | `Stop` | Steer right away from a close left wall. |
| 4 | `Close` | `Clear` | Any | `Fast` | `Fast` | Move straight fast when left wall is ideal and front is clear. |
| 5 | `Close` | `Safe` | Any | `Slow` | `Fast` | Lean left slightly while moving forward. |
| 6 | `Safe` | `Clear` | Any | `Slow` | `Fast` | Lean left to return toward wall-following distance. |
| 7 | `Safe` | `Safe` | Any | `Slow` | `Fast` | Continue left correction. |
| 8 | `Far` | `Clear` | Any | `Stop` | `Fast` | Turn left into open left space. |
| 9 | `Far` | `Safe` | Any | `Stop` | `Fast` | Turn left when left side is open and front is usable. |
| 10 | `Far` | `Clear` | `Far` | `Slow` | `Fast` | In open space, lean left to find/follow a wall. |
| 11 | Any | Any | `TooClose` | `Slow` | `Fast` | Steer left away from a close right wall. |

Generated surface plots:

| Figure | Description |
|---|---|
| `surface_left_front_vL.png` | Surface plot showing how `Left` and `Front` influence `vL`. |
| `surface_left_front_vR.png` | Surface plot showing how `Left` and `Front` influence `vR`. |

## 9. Controller Integration in `myController.m`

The controller function signature is:

```matlab
function [vL, vR] = myController(state, sensors)
```

It receives:

| Input | Meaning |
|---|---|
| `state.x` | Robot x-position in maze-cell units. |
| `state.y` | Robot y-position in maze-cell units. |
| `state.theta` | Robot heading in radians. |
| `state.v` | Current robot linear speed. |
| `state.omega` | Current angular speed. |
| `state.goalReached` | Boolean telling whether the goal has been reached. |
| `sensors.front` | Front distance sensor. |
| `sensors.left` | Left distance sensor. |
| `sensors.right` | Right distance sensor. |

It returns:

| Output | Meaning |
|---|---|
| `vL` | Commanded left wheel speed in m/s. |
| `vR` | Commanded right wheel speed in m/s. |

The controller clamps all outputs to the allowed range `[-2.8, 2.8]`.

## 10. Persistent Variables

The controller uses persistent variables so it can remember state between simulation frames.

```matlab
persistent fis mode headingTarget commitTicks commitTotal lastTurn junctionTurn filtLeft filtFront filtRight launchTicks escapeTicks escapeDir escapeSpin
```

| Variable | Purpose |
|---|---|
| `fis` | Stores the loaded fuzzy inference system so it is not reloaded every frame. |
| `mode` | Current navigation mode: `straight`, `turn`, `commit`, or `escape`. |
| `headingTarget` | Desired direction during turns and straight-line correction. |
| `commitTicks` | Number of frames to continue after a turn before allowing new decisions. |
| `commitTotal` | Original commit duration, used to blend out steering smoothly after a turn. |
| `lastTurn` | Smoothed steering memory. Prevents sudden steering jumps. |
| `junctionTurn` | Marks whether the current turn began at a T-junction. |
| `filtLeft` | Filtered left-side distance. |
| `filtFront` | Filtered front distance. |
| `filtRight` | Filtered right-side distance. |
| `launchTicks` | Short start-only stabilizer counter. |
| `escapeTicks` | Short reverse counter used during ordinary nose-on-wall escape. |
| `escapeDir` | Stored escape/spin direction. |
| `escapeSpin` | Indicates that the robot is doing an in-place 180-degree dead-end spin. |

Using persistent variables was important because a purely frame-by-frame controller had two main problems:

1. It could change its mind too often at corners or junctions.
2. It could oscillate when sensor readings changed quickly.

## 11. FIS Loading

The FIS is loaded only once:

```matlab
fisPath = fullfile(fileparts(mfilename('fullpath')), 'navigator.fis');
fis = readfis(fisPath);
```

This is more robust than calling `readfis('navigator.fis')` directly because it loads the FIS from the same folder as `myController.m`, even if MATLAB's current directory changes.

## 12. Goal Handling

When the simulator reports that the goal has been reached, the controller immediately stops both wheels:

```matlab
if state.goalReached
    vL = 0;
    vR = 0;
    ...
    return;
end
```

It also resets internal memory such as `mode`, `commitTicks`, `lastTurn`, filters, and `launchTicks`. This makes the controller behave cleanly after pressing Reset or starting another maze.

## 13. Sensor Correction

The controller swaps the side sensors before using them:

```matlab
left  = max(0, min(5, sensors.right));
front = max(0, min(5, sensors.front));
right = max(0, min(5, sensors.left));
```

This correction was kept because during testing the simulator's side labels behaved opposite to the robot-frame convention used by the controller. Without this correction, steering away from one wall could accidentally steer toward it.

All sensor values are clamped to `[0, 5]`, matching the FIS input range.

## 14. Sensor Filtering

The final version filters side and front readings:

```matlab
filtLeft  = 0.78 * filtLeft  + 0.22 * left;
filtFront = 0.55 * filtFront + 0.45 * front;
filtRight = 0.78 * filtRight + 0.22 * right;
```

The side sensors are filtered more strongly than the front sensor because side readings can fluctuate quickly near walls and corners. If the controller reacts too hard to every tiny side change, the path becomes wavy. The front sensor is filtered less strongly because front obstacles require faster response.

## 15. FIS Evaluation

The FIS is evaluated each frame:

```matlab
outputs = evalfis(fis, [filtLeft, min(front, filtFront), filtRight]);
fisSpeed = max(0, (outputs(1) + outputs(2)) / 2);
```

The FIS outputs two wheel-speed suggestions. The controller uses their average as a base speed estimate called `fisSpeed`. The final wheel commands are not simply the raw FIS outputs, because the final controller also needs memory, smoothing, and higher-level turn handling.

The expression `min(front, filtFront)` ensures that if the raw front sensor suddenly sees a close wall, the controller reacts immediately rather than waiting for the filtered value to catch up.

## 16. Control Modes

The final controller uses three main modes.

### 16.1 Straight Mode

This is the default mode. The robot moves forward quickly while correcting its heading and wall position.

In straight mode:

1. The robot aims at `headingTarget`.
2. Small wall corrections keep it away from side walls.
3. Speed can reach `2.8 m/s` in clear corridors.
4. Speed is reduced only when the front distance or side clearance becomes unsafe.

### 16.2 Turn Mode

Turn mode begins when the front wall is close enough and a side path is available:

```matlab
if strcmp(mode, 'straight') && shouldStartTurn(front, state.v, left, right)
```

The target heading is set to exactly 90 degrees from the nearest cardinal direction:

```matlab
headingTarget = wrapAngle(snapCardinal(state.theta) + turnDir * (pi/2));
```

This corrected an earlier version where the turn target was slightly less than 90 degrees. The exact 90-degree target made corridor alignment better.

During turn mode, the controller:

1. Computes angular error to the target heading.
2. Generates a turning command.
3. Adds wall correction.
4. Smooths the turn command.
5. Sends differential wheel speeds.

### 16.3 Commit Mode

After the robot finishes a turn, it briefly enters commit mode.

The purpose is to prevent hesitation. Without commit mode, the robot can finish a turn, immediately see a confusing sensor pattern, and start turning again too soon.

Final commit durations:

| Situation | `commitTicks` |
|---|---:|
| Normal turn | 24 |
| T-junction turn | 40 |

T-junctions get a longer commit because they are more likely to cause left-right oscillation.

The latest tuning also uses commit mode for post-turn centering. During this phase the robot applies a stronger side-wall correction, lowers the short commit speed to about `1.96 m/s`, and uses `commitCenterScale` to slow slightly when the left/right distances are unbalanced. It also uses `commitBlend` to carry a small part of the previous turn command into the beginning of commit mode. This makes the turn exit feel like one continuous rolling turn instead of a turn followed by a separate correction.

For U-shaped chained corners, the controller now checks `chainedTurnDirection` during commit mode. If the robot is already curving, the next front wall is approaching, and the next chosen turn direction matches the current curve, the controller enters the next `turn` mode immediately. This is intended to turn two close 90-degree actions into one smoother half-circle arc while preserving the normal stable behavior for isolated corners.

## 17. Start Stabilization

The final controller includes a short launch stabilizer:

```matlab
launchTicks = 18;
```

This only applies near the start cell:

```matlab
nearStart = state.x < 2.25 && state.y < 2.25;
```

During this start-only phase, the robot aims straight forward and limits speed to around `2.30 m/s`. This solved the problem where the robot sometimes started with a small unnecessary wiggle before it had enough corridor context.

This stabilizer is intentionally disabled once the robot leaves the start area:

```matlab
elseif ~nearStart
    launchTicks = 0;
end
```

So it does not limit speed later in the maze.

## 18. Stuck / Nose-On-Wall Escape

One important final fix was the nose-on-wall escape behavior:

```matlab
if front < 0.12
```

When the front sensor is almost zero, the robot cannot simply keep rolling forward, because the simulator's collision logic may reject movement and leave the robot stuck.

The final behavior is:

1. Choose a turn direction.
2. Turn toward the selected side.
3. Apply a small reverse speed.
4. Use differential wheel speeds to peel away from the wall.

Typical stuck-case command from MATLAB sanity testing:

```text
vL = 0.060
vR = -0.260
average speed = -0.100
```

This is not a full stop. It is a small reverse arc that creates enough clearance for the robot to escape.

## 19. Turn Direction Logic

The function `chooseTurnDirection` decides whether to turn left or right.

It first applies safety checks:

```matlab
if left < 0.55 && right >= 0.55
    dir = -1;
elseif right < 0.55 && left >= 0.55
    dir = 1;
end
```

Interpretation:

| Condition | Action |
|---|---|
| Left side is too tight, right side is usable | Turn right |
| Right side is too tight, left side is usable | Turn left |

If both sides are usable, it uses goal-aware scoring.

## 20. Goal-Aware `(x, y)` Logic

The controller does use position logic. It estimates the goal from the robot's current position:

```matlab
function goal = estimateGoal(state)
    farthestSeen = max(state.x, state.y);
    if farthestSeen > 20.5
        goal = [25, 25];
    elseif farthestSeen > 14.5
        goal = [19, 19];
    else
        goal = [13, 13];
    end
end
```

This works because the simulator uses maze sizes:

| Maze size | Goal estimate |
|---|---|
| `15x15` | `[13, 13]` |
| `21x21` | `[19, 19]` |
| `27x27` | `[25, 25]` |

The turn scoring compares the candidate left and right headings:

```matlab
candidateHeading = wrapAngle(baseHeading + dir * (pi/2));
toGoal = [goal(1) - state.x, goal(2) - state.y];
goalAlign = dot(candidateHeadingDirection, toGoalDirection);
```

The final score is:

```matlab
score = 1.25 * goalAlign + 0.12 * openness;
```

This means direction toward the goal is more important than simply choosing the more open side. Earlier, an open-side shortcut sometimes overrode the `(x, y)` logic and caused the robot to take a wrong-looking branch. That shortcut was removed. Now side openness is used only as a tie-breaker unless one side is genuinely unsafe.

## 21. Wall Correction

The function `wallCorrection` keeps the robot centered and prevents side-wall contact.

Important values:

| Parameter | Value | Meaning |
|---|---:|---|
| `corridorRange` | `1.70` | Side walls closer than this affect steering. |
| `target` | `0.62` | Desired side clearance target. |
| `deadband` | `0.07` | Ignore tiny side errors to avoid wobble. |
| `limit` | usually `0.46` | Side distance that triggers stronger protection. |

The function handles three cases:

1. Both side walls visible: center between them.
2. Only right wall close: steer left.
3. Only left wall close: steer right.

It also adds stronger correction when either side is closer than `limit`.

## 22. Speed Control

The final controller separates speed into several parts.

### 22.1 Straight Speed

Straight speed is computed using:

```matlab
speed = approachSpeed(front, fisSpeed, 2.78);
```

The cruise value is `2.78`, and the final command can reach the maximum `2.8` after boost/clamping. The latest corridor boost is slightly stronger, but it only activates when the robot is already well centered, has clear front space, and is applying almost no steering.

### 22.2 Approach Slowdown

When the front wall gets closer than `1.18 m`, speed is reduced smoothly:

```matlab
if front < 1.18
    t = clamp((front - 0.34) / (1.18 - 0.34), 0, 1);
    speed = 0.72 + t * (speed - 0.72);
end
```

This avoids hard braking and keeps the robot controllable near turns.

### 22.3 Turn Speed

The final stable turn speed uses:

```matlab
base = 0.88 + 0.34 * clamp(front / 1.3, 0, 1);
angleScale = 1.0 - 0.34 * clamp(err / (pi/2), 0, 1);
```

This means:

1. The robot carries forward motion through a turn.
2. It does not fully stop before rotating.
3. It slows when the angle error is large, so the corner radius does not become too wide.
4. It limits speed when side clearance is tight.
5. It keeps the conservative stable turn speed, preserving hall-centering behavior.

### 22.4 Side Clearance Scaling

The final side-clearance scaling is:

```matlab
if sideMin < 0.20
    scale = 0.62;
else
    scale = clamp((sideMin - 0.20) / (0.31 - 0.20), 0.80, 1.0);
end
```

This was tuned because in a normal centered corridor, side sensor readings can be around `0.32` to `0.37`. Earlier versions treated that as too dangerous and capped speed around `2.3`. The final version allows `2.8` in stable corridors while still slowing down if the robot is truly too close to a wall.

The latest straight-line tuning also adds `straightCenterScale`. This only reduces speed mildly when the left/right side distances are clearly unbalanced or when one side is very close. It gives the wall-correction loop a little more time to re-center the robot without slowing well-centered corridors.

## 23. Differential Drive Conversion

The final command form is:

```matlab
vL = clamp(speed - lastTurn, -2.8, 2.8);
vR = clamp(speed + lastTurn, -2.8, 2.8);
```

Interpretation:

| Relationship | Robot motion |
|---|---|
| `vL = vR` | Move straight |
| `vL > vR` | Turn right |
| `vR > vL` | Turn left |
| One wheel forward, one reverse | Spin/escape turn |

The variable `lastTurn` is smoothed by `smoothTurn`, so wheel commands do not jump instantly.

## 24. Simulator Acceleration

The simulator contains acceleration limits:

```matlab
ACCEL = 7.5 * CELL;
RACCEL = 6.5;
```

These were increased from lower values because the robot was commanding higher speed but physically taking too long to reach it. The final values still preserve smooth acceleration, but allow the robot to reach useful speed sooner.

The maximum wheel speed remains:

```matlab
MAX_V = 2.8 * CELL;
```

So the solution still respects the project speed limit.

## 25. Development and Tuning History

The final version was reached through iterative testing.

### 25.1 Initial FIS Behavior

The first controller relied mostly on the FIS. It could follow walls and avoid obstacles, but it had issues:

1. It sometimes turned too late.
2. It could oscillate at T-junctions.
3. It did not always use the goal direction intelligently.
4. Its speed was conservative.

### 25.2 Added Controller Modes

The `straight`, `turn`, and `commit` modes were added to prevent frame-by-frame indecision. This greatly improved stability.

### 25.3 Added Goal-Aware Turn Scoring

The `(x, y)` based turn scoring was added so the robot would prefer branches that point toward the top-right goal.

### 25.4 Added Sensor Filtering

Filtering reduced wobble, especially at high speed in corridors.

### 25.5 Added Start Stabilization

The robot sometimes struggled at the very beginning because it had little wall context and could overreact. `launchTicks` fixed the start by forcing a short straight launch.

### 25.6 Added Nose-On-Wall Escape

When the front sensor became `0.00`, the robot could get stuck because forward motion was blocked by collision. The final reverse-arc escape fixed this.

### 25.7 Increased Speed to 2.8

The controller was initially reaching only about `2.3` in corridors. The cause was side-clearance scaling, not the FIS output range. The side speed scale was adjusted so normal centered corridors can reach `2.8`.

### 25.8 Rejected Later Experimental Edit

An experimental "road-run" turn version was tested after this stable version. It made turns too wide and too sharp, so it was rejected. The working project was restored to this stable version. This folder contains the stable version, not the rejected experiment.

## 26. Final MATLAB Sanity Checks

The final stable controller was tested with representative states.

Typical sanity results:

```text
open -> vL=2.800, vR=2.800, avg=2.800
start -> vL=2.300, vR=2.300, avg=2.300
turn-entry -> vL=0.258, vR=1.058, avg=0.658, turn=0.400
stuck -> vL=0.060, vR=-0.260, avg=-0.100
```

Interpretation:

| Case | Result |
|---|---|
| Open corridor | Reaches maximum speed. |
| Start | Launches straight and stable. |
| Turn entry | Slows and turns with controlled differential speed. |
| Stuck/nose-wall | Reverses slightly while turning out. |

## 27. How to Run

Open MATLAB in this folder and run:

```matlab
runSimulation
```

`runSimulation.m` does:

```matlab
clear; clc; close all;
cd(fileparts(mfilename('fullpath')));

if ~isfile('navigator.fis')
    build_navigator_fis;
end

mazeSim(@myController);
```

So if `navigator.fis` is missing, it rebuilds it first. Then it starts the simulator with the final controller.

## 28. Report Figure References

These figures were generated from the final `navigator.fis` and can be inserted into a final PDF report.

| Figure file | Suggested report caption |
|---|---|
| `input_left_mf.png` | Membership functions for the left distance sensor. |
| `input_front_mf.png` | Membership functions for the front distance sensor. |
| `input_right_mf.png` | Membership functions for the right distance sensor. |
| `output_vL_mf.png` | Membership functions for the left wheel speed output. |
| `output_vR_mf.png` | Membership functions for the right wheel speed output. |
| `surface_left_front_vL.png` | FIS control surface showing how left/front distances affect left wheel speed. |
| `surface_left_front_vR.png` | FIS control surface showing how left/front distances affect right wheel speed. |

## 29. Suggested Final Report Structure

For a 1-2 page final report, use this compressed structure:

1. Objective: autonomous maze navigation using fuzzy logic.
2. FIS design: 3 inputs, 2 outputs, Mamdani, centroid defuzzification.
3. Membership functions: include input/output plots.
4. Rule strategy: obstacle avoidance plus wall following.
5. Controller integration: persistent FIS, modes, turn commitment, speed scaling.
6. Improvements: launch stabilization, goal-aware turn scoring, stuck escape, high-speed corridor tuning.
7. Results: stable goal reaching, maximum straight speed near `2.8 m/s`, smooth controlled turns.

## 30. Final Conclusion

The final solution keeps the required fuzzy-logic foundation while adding practical state handling needed for a real-time simulator. The FIS supplies the main obstacle-avoidance and wall-following behavior. The MATLAB controller adds stability, memory, speed control, and goal-aware decisions.

The final behavior is:

1. Stable at launch.
2. Fast in open corridors.
3. Smooth through turns.
4. Able to recover from near-wall situations.
5. Goal-directed without needing a full maze map.

This version was saved as the stable submission-ready version because it had the best balance between speed, stability, and reliability.
