# Fuzzy Logic Maze Controller Explanation

## 1. General Control Strategy

The robot controller combines a Mamdani fuzzy inference system with a small amount of extra navigation logic. The fuzzy system is stored in `navigator.fis`, while the MATLAB integration and motion smoothing are implemented in `myController.m`.

The main strategy is reactive maze navigation. The robot normally drives forward while using the side sensors to stay away from walls. When the front sensor becomes critically close to a wall, the controller enters a turn mode, rotates toward a new heading, then briefly commits to the new direction before returning to normal forward motion.

The updated version also includes a T-junction decision lock and goal-aware turn selection. When the robot reaches a T-shaped area, it chooses one branch once, scores the possible left and right turns based on which direction points closer to the known top-right goal, and then commits forward for longer after the turn. The latest tuning focuses on speed and smoothness: side sensor readings used for wall correction are filtered, and small side-distance errors are ignored so the robot does not constantly wobble left and right.

This approach was chosen because the maze is randomly generated and unknown in advance. The robot does not use a map or path planner. Instead, it reacts to the three distance sensors: left, front, and right.

## 2. Controller File: `myController.m`

### 2.1 Function Inputs and Outputs

The controller function is:

```matlab
function [vL, vR] = myController(state, sensors)
```

It receives:

| Variable | Meaning |
|---|---|
| `state.x` | Robot x-position in meters |
| `state.y` | Robot y-position in meters |
| `state.theta` | Robot heading angle in radians |
| `state.v` | Robot linear velocity |
| `state.omega` | Robot angular velocity |
| `state.goalReached` | Boolean flag indicating whether the robot reached the goal |
| `sensors.front` | Distance from the front sensor to the nearest wall |
| `sensors.left` | Distance from the left sensor to the nearest wall |
| `sensors.right` | Distance from the right sensor to the nearest wall |

It returns:

| Output | Meaning |
|---|---|
| `vL` | Left wheel speed in m/s |
| `vR` | Right wheel speed in m/s |

The wheel speed range is limited to `[-2.8, 2.8]` m/s, matching the project specification.

### 2.2 Persistent Variables

The controller uses persistent variables:

```matlab
persistent fis mode headingTarget commitTicks commitTotal lastTurn junctionTurn filtLeft filtFront filtRight launchTicks escapeTicks escapeDir escapeSpin
```

| Variable | Purpose |
|---|---|
| `fis` | Stores the fuzzy inference system after loading it once |
| `mode` | Stores the current behavior mode: `straight`, `turn`, `commit`, or `escape` |
| `headingTarget` | Desired robot heading during turns and forward correction |
| `commitTicks` | Number of frames to continue forward after a turn |
| `commitTotal` | Original commit duration, used to blend steering smoothly after a turn |
| `lastTurn` | Smoothed previous steering command |
| `junctionTurn` | Indicates that the current turn started at a T-junction |
| `filtLeft` | Filtered left-side distance used for smoother wall correction |
| `filtFront` | Filtered front distance used for smoother FIS speed response |
| `filtRight` | Filtered right-side distance used for smoother wall correction |
| `launchTicks` | Short start-only stabilizer counter |
| `escapeTicks` | Short reverse counter used during ordinary nose-on-wall escape |
| `escapeDir` | Stored escape/spin direction |
| `escapeSpin` | Indicates that the robot is completing an in-place 180-degree dead-end spin |

The `persistent` keyword prevents the `.fis` file from being reloaded every simulation frame. This improves performance and follows the project requirement.

### 2.3 Loading the FIS

At the start, the controller loads `navigator.fis`:

```matlab
fisPath = fullfile(fileparts(mfilename('fullpath')), 'navigator.fis');
fis = readfis(fisPath);
```

Using `fileparts(mfilename('fullpath'))` makes the controller load the FIS from the same folder as `myController.m`, even if MATLAB is started from another directory.

### 2.4 Goal Handling

If the robot reaches the goal, both wheel speeds are set to zero:

```matlab
if state.goalReached
    vL = 0;
    vR = 0;
    ...
    return;
end
```

The internal mode and steering memory are also reset so that the controller starts cleanly if the simulation is reset.

### 2.5 Sensor Correction

The simulator's side sensor labels are corrected before being passed to the FIS:

```matlab
left  = max(0, min(5, sensors.right));
front = max(0, min(5, sensors.front));
right = max(0, min(5, sensors.left));
```

This correction is kept because, in this simulator setup, the left and right sensor labels behave opposite to the robot frame used by the controller. The values are also clamped to the valid FIS input range `[0, 5]`.

### 2.6 FIS Evaluation

The fuzzy system is evaluated using:

```matlab
outputs = evalfis(fis, [left, front, right]);
fisSpeed = max(0, (outputs(1) + outputs(2)) / 2);
```

The FIS produces two wheel speeds. The controller uses their average as a base speed suggestion called `fisSpeed`. This keeps the fuzzy system involved in the final speed decision, while the extra controller logic improves smoothness and stability.

## 3. Controller Modes

### 3.1 Straight Mode

This is the default mode. The robot moves forward and applies small steering corrections based on heading error and wall distance.

The robot only enters turn mode when the front wall is very close:

```matlab
if strcmp(mode, 'straight') && front < 0.38
```

This value was tuned to avoid turning too early. Earlier turn thresholds made the robot rotate before it reached the corridor end, which caused unstable behavior. The final version keeps the successful reactive behavior and focuses on smoother motion instead of earlier turns.

Before entering turn mode, the controller checks whether the situation looks like a T-junction:

```matlab
junctionTurn = isTJunction(front, left, right);
```

A T-junction is detected when the front is blocked but both side directions are open:

```matlab
front < 0.45 && left > 0.75 && right > 0.75
```

This does not make the robot turn earlier. It only marks the turn so the controller can commit for longer after choosing a branch.

### 3.2 Turn Mode

When the front is blocked, the controller chooses a turn direction and rotates toward a target heading:

```matlab
headingTarget = wrapAngle(snapCardinal(state.theta) + turnDir * 1.43);
```

The value `1.43` radians is slightly less than 90 degrees. This makes the robot turn enough to escape a blocked front wall without producing overly sharp turns.

The turn direction is selected by `chooseTurnDirection`. The controller first avoids directions that are too tight. Then it estimates the goal location and scores the possible left and right turns.

```matlab
goal = estimateGoal(state);
leftScore = turnScore(baseHeading, 1, state, goal, left);
rightScore = turnScore(baseHeading, -1, state, goal, right);
```

The goal is estimated from the maze size:

| Maze size | Approximate goal used by controller |
|---|---|
| `15x15` | `(13, 13)` |
| `21x21` | `(19, 19)` |
| `27x27` | `(25, 25)` |

The scoring checks which candidate heading points more toward the goal. It also adds a small score for side openness so the robot still prefers safer, wider branches. If the two scores are almost equal, the controller uses side openness as a tie-breaker, then a small right preference as the final fallback.

In this controller, `dir = -1` means a right turn and `dir = +1` means a left turn. The robot is not pure right-wall-following anymore. Instead, it prefers the branch that is both open and directionally useful for reaching the top-right goal.

During the turn:

```matlab
turn = clamp(0.80 * err, -0.60, 0.60);
```

This limits the maximum turn command while allowing controlled 90-degree rotations. A minimum turn value is also used so the robot does not stall near the target angle. The latest tuning keeps the turn close to the stable version, with only a small angular increase so hall-centering behavior is preserved. The turn command is still ramped using `smoothTurn`, so the robot does not jump instantly to maximum angular speed.

### 3.3 Commit Mode

After turning, the robot enters a short commit phase:

```matlab
commitTicks = 24;
```

This phase makes the robot continue in the new direction briefly instead of immediately switching back to normal wall following. It prevents hesitation and repeated turning at corners.

If the turn started at a T-junction, the commit time is longer:

```matlab
commitTicks = 40;
```

This longer commit is the main fix for T-junction oscillation. Without it, the robot can enter the horizontal part of a T, repeatedly see both side options, and bounce left-right-left. The longer commit forces the robot to finish the chosen branch before it is allowed to make another turn decision.

The latest version also uses commit mode to center the robot after a turn. It applies a stronger wall correction, lowers the short commit speed, and slows slightly when the side distances are unbalanced so the robot returns to the middle of the hall before accelerating. It also carries a small part of the previous turn command into the start of commit mode using `commitBlend`, which makes the corner exit behave like one continuous rolling turn instead of two separate steering actions.

For U-shaped chained corners, the controller checks `chainedTurnDirection` during commit mode. If the robot is already curving, another front wall is approaching, and the next turn direction matches the current curve, it starts the next turn immediately. This helps two close 90-degree turns form a smoother half-circle arc.

If the front becomes blocked again, the controller exits commit mode early:

```matlab
if front < 0.36
    mode = 'straight';
end
```

## 4. Smoothing and Speed Control

### 4.1 Wall Correction

The helper function `wallCorrection` keeps the robot away from side walls.

Important values:

| Parameter | Value | Meaning |
|---|---:|---|
| `corridorRange` | `1.55` | Distance below which side walls are considered relevant |
| `target` | `0.52` | Desired approximate side clearance |
| `limit` | `0.44` or `0.46` | Critical side clearance used for extra correction |

When both side walls are visible, the controller tries to center the robot between them. When only one side wall is close, it gently steers away from that wall.

### 4.2 Turn Smoothing

The helper function `smoothTurn` prevents sudden steering jumps:

```matlab
y = prev + clamp(desired - prev, -maxStep, maxStep);
```

This acts like steering damping. It makes the robot path smoother and reduces oscillation in corridors.

### 4.3 Speed Selection

The helper function `approachSpeed` computes the forward speed:

```matlab
speed = clamp(0.26 * fisSpeed + cruise, 0.88, cruise);
```

The robot cruises faster when the front path is clear. The updated controller raises the straight-line cruise target to about `2.46 m/s`, with an extra corridor boost that can raise the command to about `2.76 m/s` in long, clear, straight passages. It still keeps lower speeds in commit mode and turn mode. If the front distance becomes smaller than about `1.32 m`, the speed is reduced gradually:

```matlab
t = clamp((front - 0.38) / (1.32 - 0.38), 0, 1);
speed = 0.34 + t * (speed - 0.34);
```

This avoids harsh braking and helps the robot approach corners more smoothly. Speed is also reduced when the steering command is large, so the robot mainly reaches the higher speed in straight corridors with enough front clearance.

The boost is only applied when the front is clear, the robot is well balanced between the side walls, and the steering command is almost zero:

```matlab
if front > 1.85 && min(left, right) > 0.30 && abs(left - right) < 0.32 && abs(turn) < 0.030
```

Finally, a side-clearance scale reduces speed when the robot is very close to a side wall, but it no longer over-throttles stable straight corridors. This lets the robot move close to the allowed maximum speed in open areas without becoming reckless in tight corners. The latest straight-line tuning also adds `straightCenterScale`, which slows only mildly when the side distances are clearly unbalanced so the robot can re-center without losing speed in well-centered corridors.

For turn mode, the latest version keeps a conservative rolling speed:

```matlab
base = 0.88 + 0.34 * clamp(front / 1.3, 0, 1);
angleScale = 1.0 - 0.34 * clamp(err / (pi/2), 0, 1);
```

This avoids the wide turning curve from carrying too much forward speed. The side-clearance caps still slow the robot near walls, which helps it finish closer to the middle of the hall.

To reduce wobble at high speed, the controller filters the side sensor readings used for wall correction:

```matlab
filtLeft = 0.78 * filtLeft + 0.22 * left;
filtRight = 0.78 * filtRight + 0.22 * right;
```

The wall correction function also uses a small deadband of about `0.12 m`. This means tiny side-distance differences are ignored instead of causing constant small steering reversals. The heading correction is kept slightly stronger than the side-wall correction, so the robot prefers staying aligned with the maze corridor instead of chasing every sensor fluctuation.

### 4.4 Wheel Speed Conversion

The final wheel speeds are computed as:

```matlab
vL = clamp(speed - lastTurn, -2.8, 2.8);
vR = clamp(speed + lastTurn, -2.8, 2.8);
```

This follows differential-drive behavior:

| Condition | Result |
|---|---|
| `vL = vR` | Robot moves straight |
| `vL > vR` | Robot turns right |
| `vR > vL` | Robot turns left |

## 5. FIS File: `navigator.fis`

The fuzzy inference system is a Mamdani FIS named `navigator`.

| Property | Value |
|---|---|
| FIS type | Mamdani |
| Number of inputs | 3 |
| Number of outputs | 2 |
| Number of rules | 11 |
| AND method | Minimum |
| OR method | Maximum |
| Implication method | Minimum |
| Aggregation method | Maximum |
| Defuzzification method | Centroid |

## 6. FIS Inputs

### 6.1 Input 1: Left

Range: `[0, 5]` meters

| MF index | Name | Type | Parameters |
|---:|---|---|---|
| 1 | `TooClose` | triangular | `[0, 0, 0.6]` |
| 2 | `Close` | triangular | `[0.4, 0.8, 1.4]` |
| 3 | `Safe` | triangular | `[1.0, 1.8, 2.6]` |
| 4 | `Far` | triangular | `[2.0, 5.0, 5.0]` |

### 6.2 Input 2: Front

Range: `[0, 5]` meters

| MF index | Name | Type | Parameters |
|---:|---|---|---|
| 1 | `TooClose` | triangular | `[0, 0, 0.6]` |
| 2 | `Close` | triangular | `[0.4, 1.0, 1.8]` |
| 3 | `Safe` | triangular | `[1.4, 2.5, 3.6]` |
| 4 | `Clear` | triangular | `[3.0, 5.0, 5.0]` |

### 6.3 Input 3: Right

Range: `[0, 5]` meters

| MF index | Name | Type | Parameters |
|---:|---|---|---|
| 1 | `TooClose` | triangular | `[0, 0, 0.6]` |
| 2 | `Close` | triangular | `[0.4, 0.8, 1.4]` |
| 3 | `Safe` | triangular | `[1.0, 1.8, 2.6]` |
| 4 | `Far` | triangular | `[2.0, 5.0, 5.0]` |

## 7. FIS Outputs

### 7.1 Output 1: vL

Range: `[-2.8, 2.8]` m/s

| MF index | Name | Type | Parameters |
|---:|---|---|---|
| 1 | `Reverse` | triangular | `[-2.8, -2.8, -1.2]` |
| 2 | `Stop` | triangular | `[-1.5, 0.0, 1.5]` |
| 3 | `Slow` | triangular | `[0.5, 1.4, 2.2]` |
| 4 | `Fast` | triangular | `[1.8, 2.8, 2.8]` |

### 7.2 Output 2: vR

Range: `[-2.8, 2.8]` m/s

| MF index | Name | Type | Parameters |
|---:|---|---|---|
| 1 | `Reverse` | triangular | `[-2.8, -2.8, -1.2]` |
| 2 | `Stop` | triangular | `[-1.5, 0.0, 1.5]` |
| 3 | `Slow` | triangular | `[0.5, 1.4, 2.2]` |
| 4 | `Fast` | triangular | `[1.8, 2.8, 2.8]` |

## 8. FIS Rule Table

In the `.fis` file, a `0` input means "don't care". This means the rule can fire regardless of that sensor's membership value.

| Rule | Left condition | Front condition | Right condition | vL output | vR output | Meaning |
|---:|---|---|---|---|---|---|
| 1 | Any | `TooClose` | Any | `Fast` | `Reverse` | If the front wall is extremely close, spin right to avoid collision. |
| 2 | Any | `Close` | Any | `Fast` | `Stop` | If the front is close, turn right by moving the left wheel faster. |
| 3 | `TooClose` | Any | Any | `Fast` | `Stop` | If the left side is too close, steer right away from the wall. |
| 4 | `Close` | `Clear` | Any | `Fast` | `Fast` | If left wall distance is ideal and front is clear, move straight fast. |
| 5 | `Close` | `Safe` | Any | `Slow` | `Fast` | If left is ideal and front is safe, lean slightly left while moving. |
| 6 | `Safe` | `Clear` | Any | `Slow` | `Fast` | If the left wall is farther than ideal, steer left gently. |
| 7 | `Safe` | `Safe` | Any | `Slow` | `Fast` | If the left wall is safe and the front is safe, keep leaning left. |
| 8 | `Far` | `Clear` | Any | `Stop` | `Fast` | If there is open space on the left and front is clear, turn left harder. |
| 9 | `Far` | `Safe` | Any | `Stop` | `Fast` | If the left side is far and the front is safe, turn left into the opening. |
| 10 | `Far` | `Clear` | `Far` | `Slow` | `Fast` | If all directions are open, lean left to search for/follow a wall. |
| 11 | Any | Any | `TooClose` | `Slow` | `Fast` | If the right side is too close, steer left away from the wall. |

## 9. Rule Strategy Summary

The FIS mainly implements a left-wall-following behavior with obstacle avoidance:

1. If the front is blocked, the robot turns right to avoid collision.
2. If the left wall is close but not dangerous, the robot moves forward.
3. If the left wall is too close, the robot steers right.
4. If the left wall is far, the robot steers left to find or follow the wall.
5. If the right wall is too close, the robot steers left for protection.

The MATLAB controller then improves this fuzzy behavior by adding:

1. Mode memory, so the robot does not change behavior every frame.
2. Turn commitment, so it completes turns more reliably.
3. T-junction locking, so it chooses one branch and does not oscillate left and right.
4. Goal-aware branch choice, useful because the goal is always in the top-right corner.
5. Filtered side-wall correction, so the robot wobbles less at high speed.
6. Steering smoothing, so the path is less wavy.
7. Speed scaling, so it moves faster in clear corridors and slows down near walls.
8. Wheel speed clamping, so all outputs remain inside the allowed motor range.

## 10. Final Result

The final controller keeps the project fully based on fuzzy logic while adding practical smoothing and state handling. The FIS provides the main navigation behavior, and `myController.m` adapts it into stable wheel commands for the simulator. The result is a robot that can reach the goal reliably, move faster in straight corridors, choose more goal-directed branches, avoid overly sharp or premature turns, and reduce left-right oscillation at T-junctions.
