function [vL, vR] = myController(state, sensors)
    persistent fis mode headingTarget commitTicks commitTotal lastTurn junctionTurn filtLeft filtFront filtRight launchTicks escapeTicks escapeDir escapeSpin
    if isempty(fis)
        fisPath = fullfile(fileparts(mfilename('fullpath')), 'navigator.fis');
        fis = readfis(fisPath);
        mode = 'straight';
        headingTarget = snapCardinal(state.theta);
        commitTicks = 0;
        commitTotal = 0;
        lastTurn = 0;
        junctionTurn = false;
        filtLeft = NaN;
        filtFront = NaN;
        filtRight = NaN;
        launchTicks = 18;
        escapeTicks = 0;
        escapeDir = 1;
        escapeSpin = false;
    end

    if state.goalReached
        vL = 0; vR = 0;
        mode = 'straight';
        headingTarget = snapCardinal(state.theta);
        commitTicks = 0;
        commitTotal = 0;
        lastTurn = 0;
        junctionTurn = false;
        filtLeft = NaN;
        filtFront = NaN;
        filtRight = NaN;
        launchTicks = 18;
        escapeTicks = 0;
        escapeDir = 1;
        escapeSpin = false;
        return;
    end

    % In this simulator the side sensor labels are swapped relative to the
    % robot frame, so keep this small correction before feeding the FIS.
    left  = max(0, min(5, sensors.right));
    front = max(0, min(5, sensors.front));
    right = max(0, min(5, sensors.left));

    if isnan(filtLeft)
        filtLeft = left;
        filtFront = front;
        filtRight = right;
    else
        filtLeft = 0.78 * filtLeft + 0.22 * left;
        filtFront = 0.55 * filtFront + 0.45 * front;
        filtRight = 0.78 * filtRight + 0.22 * right;
    end

    outputs = evalfis(fis, [filtLeft, min(front, filtFront), filtRight]);
    fisSpeed = max(0, (outputs(1) + outputs(2)) / 2);

    if state.x < 1.85 && state.y < 1.85 && state.v < 0.08
        mode = 'straight';
        headingTarget = 0;
        commitTicks = 0;
        commitTotal = 0;
        lastTurn = 0;
        launchTicks = 18;
        escapeTicks = 0;
        escapeSpin = false;
    end

    turnDir = chooseTurnDirection(state, left, right);

    if strcmp(mode, 'escape')
        if escapeTicks > 0 && ~escapeSpin
            escapeTicks = escapeTicks - 1;
            turn = 0;
            if min(left, right) > 0.20
                turn = 0.08 * escapeDir;
            end
            lastTurn = smoothTurn(lastTurn, turn, 0.45, 0.08);
            speed = -0.34;
            vL = clamp(speed - lastTurn, -2.8, 2.8);
            vR = clamp(speed + lastTurn, -2.8, 2.8);
            return;
        end

        err = wrapAngle(headingTarget - state.theta);
        if abs(err) < 0.09 && (front > 0.28 || escapeSpin)
            mode = 'commit';
            commitTicks = 20;
            commitTotal = commitTicks;
            junctionTurn = false;
            escapeSpin = false;
        else
            turn = clamp(0.82 * err, -0.64, 0.64);
            if abs(turn) < 0.18
                turn = 0.18 * sign(err);
            end
            if escapeSpin
                lastTurn = smoothTurn(lastTurn, turn, 0.85, 0.24);
                speed = 0;
            else
                lastTurn = smoothTurn(lastTurn, turn, 0.72, 0.20);
                speed = 0.10;
                if front < 0.18
                    speed = -0.06;
                end
            end
            vL = clamp(speed - lastTurn, -2.8, 2.8);
            vR = clamp(speed + lastTurn, -2.8, 2.8);
            return;
        end
    end

    if isDeadEnd(front, left, right)
        escapeDir = chooseSpinDirection(state, left, right);
        headingTarget = wrapAngle(snapCardinal(state.theta) + escapeDir * (pi - 0.001));
        mode = 'escape';
        escapeSpin = true;
        escapeTicks = 0;
        lastTurn = smoothTurn(lastTurn, 0.46 * escapeDir, 0.90, 0.24);

        vL = clamp(-lastTurn, -2.8, 2.8);
        vR = clamp(lastTurn, -2.8, 2.8);
        return;
    end

    if front < 0.14
        escapeDir = turnDir;
        if left < 0.55 && right < 0.55
            escapeDir = chooseSpinDirection(state, left, right);
            headingTarget = wrapAngle(snapCardinal(state.theta) + escapeDir * (pi - 0.001));
            escapeSpin = true;
        else
            headingTarget = wrapAngle(snapCardinal(state.theta) + escapeDir * (pi/2));
            escapeSpin = false;
        end
        mode = 'escape';
        escapeTicks = 8 + 8 * (min(left, right) < 0.22);
        lastTurn = smoothTurn(lastTurn, 0, 0.75, 0.18);

        if escapeSpin
            vL = clamp(-lastTurn, -2.8, 2.8);
            vR = clamp(lastTurn, -2.8, 2.8);
        else
            speed = -0.34;
            vL = clamp(speed - lastTurn, -2.8, 2.8);
            vR = clamp(speed + lastTurn, -2.8, 2.8);
        end
        return;
    end

    % Start corners early enough for a rolling turn. At T-junctions, choose
    % one branch once and commit so the robot does not hesitate mid-junction.
    if strcmp(mode, 'straight') && shouldStartTurn(front, state.v, left, right)
        junctionTurn = isTJunction(front, left, right);
        headingTarget = wrapAngle(snapCardinal(state.theta) + turnDir * (pi/2));
        mode = 'turn';
        lastTurn = 0.22 * turnDir;
        launchTicks = 0;
    end

    if strcmp(mode, 'turn')
        err = wrapAngle(headingTarget - state.theta);
        if abs(err) < 0.075
            mode = 'commit';
            if junctionTurn
                commitTicks = 40;
            else
                commitTicks = 24;
            end
            commitTotal = commitTicks;
            junctionTurn = false;
        else
            turn = clamp(0.80 * err, -0.60, 0.60);
            if abs(turn) < 0.13
                turn = 0.14 * sign(err);
            end
            turn = turn + wallCorrection(left, right, 0.46, 0.058);
            turn = clamp(turn, -0.67, 0.67);

            speed = turnSpeed(front, left, right, abs(err));
            lastTurn = smoothTurn(lastTurn, turn, 0.69, turnRamp(abs(err), left, right));
            vL = clamp(speed - lastTurn, -2.8, 2.8);
            vR = clamp(speed + lastTurn, -2.8, 2.8);
            return;
        end
    end

    if strcmp(mode, 'commit')
        if front < 0.36
            mode = 'straight';
        else
            chainDir = chainedTurnDirection(state, front, left, right, lastTurn);
            if chainDir ~= 0
                headingTarget = wrapAngle(snapCardinal(state.theta) + chainDir * (pi/2));
                mode = 'turn';
                junctionTurn = false;
                launchTicks = 0;
                lastTurn = smoothTurn(lastTurn, 0.30 * chainDir, 0.65, 0.10);

                turn = clamp(0.62 * chainDir + wallCorrection(left, right, 0.48, 0.052), -0.66, 0.66);
                lastTurn = smoothTurn(lastTurn, turn, 0.64, turnRamp(pi/2, left, right));
                speed = turnSpeed(front, left, right, pi/2);
                speed = max(speed, 0.86);
                speed = speed * sideClearanceScale(left, right);

                vL = clamp(speed - lastTurn, -2.8, 2.8);
                vR = clamp(speed + lastTurn, -2.8, 2.8);
                return;
            end

            commitTicks = commitTicks - 1;
            if commitTicks <= 0
                mode = 'straight';
            end

            err = wrapAngle(headingTarget - state.theta);
            blend = commitBlend(commitTicks, commitTotal);
            turn = 0.38 * err + wallCorrection(left, right, 0.50, 0.088) + 0.34 * blend * lastTurn;
            turn = clamp(turn, -0.24, 0.24);
            lastTurn = smoothTurn(lastTurn, turn, 0.32, turnStep(left, right, 0.046, 0.085));
            speed = approachSpeed(front, fisSpeed, 1.96);
            speed = speed * commitCenterScale(left, right);
            speed = speed * sideClearanceScale(left, right);

            vL = clamp(speed - lastTurn, -2.8, 2.8);
            vR = clamp(speed + lastTurn, -2.8, 2.8);
            return;
        end
    end

    nearStart = state.x < 2.25 && state.y < 2.25;
    if launchTicks > 0 && nearStart && front > 0.70
        launchTicks = launchTicks - 1;
        headingTarget = 0;
        err = wrapAngle(headingTarget - state.theta);
        turn = 0.34 * err + wallCorrection(left, right, 0.42, 0.035);
        turn = clamp(turn, -0.075, 0.075);
        lastTurn = smoothTurn(lastTurn, turn, 0.20, 0.018);
        speed = approachSpeed(front, fisSpeed, 2.30);
        speed = speed * sideClearanceScale(left, right);

        vL = clamp(speed - lastTurn, -2.8, 2.8);
        vR = clamp(speed + lastTurn, -2.8, 2.8);
        return;
    elseif ~nearStart
        launchTicks = 0;
    end

    err = wrapAngle(headingTarget - state.theta);
    turn = 0.50 * err + wallCorrection(left, right, 0.50, 0.078);
    turn = clamp(turn, -0.18, 0.18);
    lastTurn = smoothTurn(lastTurn, turn, 0.24, turnStep(left, right, 0.028, 0.070));

    speed = approachSpeed(front, fisSpeed, 2.78);
    speed = speed * turnSpeedScale(lastTurn);
    speed = speed + corridorBoost(front, left, right, lastTurn);
    speed = speed * straightCenterScale(left, right);
    speed = speed * sideClearanceScale(left, right);

    vL = clamp(speed - lastTurn, -2.8, 2.8);
    vR = clamp(speed + lastTurn, -2.8, 2.8);
end

function a = wrapAngle(a)
    a = atan2(sin(a), cos(a));
end

function a = snapCardinal(a)
    a = round(a / (pi/2)) * (pi/2);
    a = wrapAngle(a);
end

function tf = shouldStartTurn(front, speed, left, right)
    sideOpen = max(left, right) > 0.62 || front < 0.34;
    openSide = max(left, right);
    trigger = 0.47 + 0.16 * clamp(speed / 2.8, 0, 1);
    if openSide > 1.05
        trigger = trigger + 0.04;
    end
    tf = front < trigger && sideOpen;
end

function turn = wallCorrection(left, right, limit, gain)
    corridorRange = 1.70;
    target = 0.62;
    deadband = 0.07;

    if left < corridorRange && right < corridorRange
        sideError = left - right;
        if abs(sideError) < deadband
            turn = 0;
        else
            turn = gain * (sideError - deadband * sign(sideError)) / target;
        end
    elseif right < corridorRange
        sideError = target - right;
        if abs(sideError) < deadband
            turn = 0;
        else
            turn = gain * (sideError - deadband * sign(sideError)) / target;
        end
    elseif left < corridorRange
        sideError = target - left;
        if abs(sideError) < deadband
            turn = 0;
        else
            turn = -gain * (sideError - deadband * sign(sideError)) / target;
        end
    else
        turn = 0;
    end

    if right < limit
        turn = turn + 0.125 * (limit - right) / limit;
    end
    if left < limit
        turn = turn - 0.125 * (limit - left) / limit;
    end
end

function dir = chooseTurnDirection(state, left, right)
    if left < 0.55 && right >= 0.55
        dir = -1;
        return;
    elseif right < 0.55 && left >= 0.55
        dir = 1;
        return;
    end

    goal = estimateGoal(state);
    baseHeading = snapCardinal(state.theta);
    leftScore = turnScore(baseHeading, 1, state, goal, left);
    rightScore = turnScore(baseHeading, -1, state, goal, right);

    if abs(leftScore - rightScore) < 0.08
        sideDiff = left - right;
        if abs(sideDiff) > 0.28
            dir = sign(sideDiff);
        else
            dir = -1;
        end
    elseif leftScore > rightScore
        dir = 1;
    else
        dir = -1;
    end
end

function tf = isTJunction(front, left, right)
    tf = front < 0.45 && left > 0.75 && right > 0.75;
end

function tf = isDeadEnd(front, left, right)
    sideMin = min(left, right);
    sideMax = max(left, right);
    tf = (front < 0.20 && sideMin < 0.18) || ...
         (front < 0.26 && sideMax < 0.62);
end

function dir = chooseSpinDirection(state, left, right)
    if left > right + 0.12
        dir = 1;
    elseif right > left + 0.12
        dir = -1;
    else
        goal = estimateGoal(state);
        baseHeading = snapCardinal(state.theta);
        leftScore = turnScore(baseHeading, 1, state, goal, left);
        rightScore = turnScore(baseHeading, -1, state, goal, right);
        if leftScore >= rightScore
            dir = 1;
        else
            dir = -1;
        end
    end
end

function speed = approachSpeed(front, fisSpeed, cruise)
    speed = clamp(0.08 * fisSpeed + cruise, 1.05, cruise);
    if front < 1.18
        t = clamp((front - 0.34) / (1.18 - 0.34), 0, 1);
        speed = 0.72 + t * (speed - 0.72);
    end
end

function speed = turnSpeed(front, left, right, err)
    clearance = min(left, right);
    base = 0.88 + 0.34 * clamp(front / 1.3, 0, 1);
    angleScale = 1.0 - 0.34 * clamp(err / (pi/2), 0, 1);
    speed = base * angleScale;

    if clearance < 0.34
        speed = min(speed, 0.78);
    elseif clearance < 0.44
        speed = min(speed, 0.86);
    elseif clearance < 0.58
        speed = min(speed, 1.02);
    end
end

function step = turnRamp(err, left, right)
    if min(left, right) < 0.36
        step = 0.26;
    elseif err > 0.65
        step = 0.19;
    else
        step = 0.12;
    end
end

function step = turnStep(left, right, normalStep, urgentStep)
    if min(left, right) < 0.36
        step = urgentStep;
    else
        step = normalStep;
    end
end

function dir = chainedTurnDirection(state, front, left, right, lastTurn)
    dir = 0;
    if abs(lastTurn) < 0.08 || front > 0.82 || max(left, right) < 0.82
        return;
    end

    candidate = chooseTurnDirection(state, left, right);
    if sign(lastTurn) * candidate > 0
        dir = candidate;
    end
end

function blend = commitBlend(ticksLeft, totalTicks)
    if totalTicks <= 0
        blend = 0;
    else
        phase = ticksLeft / totalTicks;
        blend = clamp((phase - 0.45) / 0.40, 0, 1);
    end
end

function scale = commitCenterScale(left, right)
    sideMin = min(left, right);
    imbalance = abs(left - right);
    scale = 1.0 - 0.20 * clamp(imbalance / 0.45, 0, 1);
    if sideMin < 0.28
        scale = min(scale, 0.78);
    end
end

function scale = turnSpeedScale(turn)
    scale = 1.0 - min(0.14, 0.55 * abs(turn));
end

function boost = corridorBoost(front, left, right, turn)
    sideBalanced = abs(left - right) < 0.32;
    sideUsable = min(left, right) > 0.30;
    if front > 1.85 && sideUsable && sideBalanced && abs(turn) < 0.030
        boost = 0.12 * clamp((front - 1.85) / 1.10, 0, 1);
    else
        boost = 0;
    end
end

function scale = straightCenterScale(left, right)
    sideMin = min(left, right);
    imbalance = abs(left - right);
    scale = 1.0 - 0.10 * clamp((imbalance - 0.32) / 0.55, 0, 1);
    if sideMin < 0.24
        scale = min(scale, 0.82);
    end
end

function scale = sideClearanceScale(left, right)
    sideMin = min(left, right);
    if sideMin < 0.20
        scale = 0.62;
    else
        scale = clamp((sideMin - 0.20) / (0.31 - 0.20), 0.80, 1.0);
    end
end

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

function score = turnScore(baseHeading, dir, state, goal, sideOpen)
    candidateHeading = wrapAngle(baseHeading + dir * (pi/2));
    toGoal = [goal(1) - state.x, goal(2) - state.y];
    distGoal = hypot(toGoal(1), toGoal(2));
    if distGoal < 0.001
        goalAlign = 0;
    else
        goalAlign = (cos(candidateHeading) * toGoal(1) + sin(candidateHeading) * toGoal(2)) / distGoal;
    end

    openness = clamp(sideOpen / 2.5, 0, 1);
    score = 1.25 * goalAlign + 0.12 * openness;
    if sideOpen < 0.55
        score = score - 3.0;
    end
end

function y = smoothTurn(prev, target, alpha, maxStep)
    desired = (1 - alpha) * prev + alpha * target;
    y = prev + clamp(desired - prev, -maxStep, maxStep);
end

function y = clamp(x, lo, hi)
    y = max(lo, min(hi, x));
end
