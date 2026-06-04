function mazeSim(controllerFcn)
% mazeSim  Differential Drive Robot – Modern UI Dashboard
%
% Usage:
%   mazeSim(@myController)  % Runs with your custom controller available

    if nargin < 1 || ~isa(controllerFcn, 'function_handle')
        controllerFcn = @defaultAutoController;
    end
    
    useExternal = true; % Start in automatic mode

    %% ── 1. Modern Design Token Palette (Tailwind Inspired) ──────────────
    C.bg         = [15,  23,  42 ]/255; % Slate 900 (Deepest background)
    C.card       = [30,  41,  59 ]/255; % Slate 800 (Elevated panels)
    C.cardHover  = [51,  65,  85 ]/255; % Slate 700 (Button hovers)
    C.border     = [71,  85,  105]/255; % Slate 600 (Subtle dividers)
    
    C.text       = [248, 250, 252]/255; % Slate 50  (Primary text)
    C.textMuted  = [148, 163, 184]/255; % Slate 400 (Secondary text)
    
    C.primary    = [20,  184, 166]/255; % Teal 500  (Active/Success)
    C.primaryDim = [13,  148, 136]/255; % Teal 600  (Hover)
    C.warning    = [244, 63,  94 ]/255; % Rose 500  (Alerts/Goal)
    
    C.robot      = [250, 204, 21 ]/255; % Yellow 400
    C.wheel      = [161, 161, 170]/255; % Zinc 400
    C.trail      = [56,  189, 248]/255; % Sky 400
    C.wall       = [30,  41,  59 ]/255; % Same as card
    C.passage    = [15,  23,  42 ]/255; % Same as bg

    fontSans = 'sans-serif'; % Modern UI font
    fontMono = 'sans-serif'; % Data font

    %% ── 2. Simulation State ─────────────────────────────────────────────────
    SIZES = [15, 21, 27]; sizeIdx = 1; maze = [];
    CELL = 3.0; 
    
    robot    = struct('x', 1.5*CELL, 'y', 1.5*CELL, 'theta', 0, 'v', 0, 'omega', 0);
    startPos = struct('x', 1.5*CELL, 'y', 1.5*CELL);
    goalPos  = struct('x', 0, 'y', 0); 
    
    goalReached = false; elapsedTime = 0; totalDist = 0;
    prevX = robot.x; prevY = robot.y;

    Lr = 1.60; Wr = 1.10; HWB = 0.52;  

    MAX_V = 2.8 * CELL; MAX_OM = 2.2;
    ACCEL = 7.5 * CELL; RACCEL = 6.5;
    LIN_DAMP = 6.0; ANG_DAMP = 7.0;

    TRAIL_MAX = 10000; trailX = nan(1, TRAIL_MAX); trailY = nan(1, TRAIL_MAX);
    trailPtr = 0; showTrail = true;

    K = struct('w',false,'s',false,'a',false,'d',false,...
               'up',false,'dn',false,'lt',false,'rt',false);

    %% ── 3. Figure & Layout Setup ────────────────────────────────────────────
    fig = uifigure('Name','Maze Simulator',...
        'Icon','maze.png',...
        'Scrollable','on',...
        'Position',[60 60 1200 800],'Color',C.bg,...
        'WindowKeyPressFcn', @(~,e) onKey(e.Key, true),...
        'WindowKeyReleaseFcn',@(~,e) onKey(e.Key, false)); 

    mainGrid = uigridlayout(fig,[3,1],...
        'RowHeight',{64,'1x',50},'ColumnWidth',{'1x'},...
        'BackgroundColor',C.bg,'RowSpacing',0,'Padding',[0 0 0 0]);

    % --- TOP NAVIGATION BAR ---
    topNav = uigridlayout(mainGrid,[1,7],...
        'BackgroundColor',C.card,...
        'ColumnWidth',{220,'1x', 150, 120, 120,120, 120},...
        'Padding',[24 10 18 10],'ColumnSpacing',10);
    
    uilabel(topNav,'Text','𖦹 Maze Runner',...
        'FontColor',C.primary,'FontWeight','bold','FontSize',20,'FontName',fontSans);

    % Fixed: Removed BorderColor
    btnMode = uibutton(topNav,'Text','🤖 MODE: AUTO',...
        'BackgroundColor',C.cardHover,'FontColor',C.text,'FontWeight','bold',...
        'FontSize', 14, 'FontName', fontSans,...
        'ButtonPushedFcn',@(~,~) toggleMode());
    btnMode.Layout.Column = 3;

    btnNew = uibutton(topNav,'Text', '🎲 New Maze','BackgroundColor',C.cardHover,'FontColor',C.text,'FontWeight','bold',...
        'FontSize', 14, 'FontName', fontSans,...
        'ButtonPushedFcn', @(~,~) initSim(true)); btnNew.Layout.Column = 4;
    btnReset = uibutton(topNav,'Text', '🔄 Reset','BackgroundColor',C.cardHover,'FontColor',C.text,'FontWeight','bold',...
        'FontSize', 14, 'FontName', fontSans,...
        'ButtonPushedFcn', @(~,~) resetRobot()); btnReset.Layout.Column = 5;
    btnTrail = uibutton(topNav,'Text','👣 Trail ON',...
        'BackgroundColor',C.primaryDim,'FontColor',C.text,'FontWeight','bold',...
        'FontSize', 14, 'FontName', fontSans,...
        'ButtonPushedFcn',@(~,~) toggleTrail());
    btnTrail.Layout.Column = 6;
    btnSz = uibutton(topNav,'Text', '📐 Size: 15x15','BackgroundColor',C.cardHover,'FontColor',C.text,'FontWeight','bold',...
        'FontSize', 14, 'FontName', fontSans,...
        'ButtonPushedFcn', @(~,~) changeSize()); btnSz.Layout.Column = 7;

    % --- MAIN CONTENT AREA ---
    contentGrid = uigridlayout(mainGrid,[1,2],...
        'ColumnWidth',{'1x', 360},'RowHeight',{'1x'},...
        'BackgroundColor',C.bg,'Padding',[10 10 10 10],'ColumnSpacing',24);

    % Canvas
    canvPanel = uipanel(contentGrid,'BackgroundColor',C.bg,'BorderType','none');
    ax = uiaxes(canvPanel,'Units','normalized','Position',[0 0 1 1]);
    ax.Color = C.bg; ax.XColor = 'none'; ax.YColor = 'none';
    ax.DataAspectRatio = [1 1 1];
    ax.PositionConstraint="innerposition";
    hold(ax,'on'); disableDefaultInteractivity(ax);


    % --- SIDEBAR (CARDS) ---
    sidebar = uigridlayout(contentGrid,[4,1],...
        'BackgroundColor',C.bg,...
        'RowHeight',{'1.1x', '0.7x', 170, '1x'},...
        'Padding',[0 0 0 0],'RowSpacing',16);

    % Card 1: Robot Pose
    cP = uigridlayout(sidebar,[3,1], ...
        'BackgroundColor',C.card,...
        'RowHeight',{25, '1x', '1x'},...
        'Padding',[10 10 10 10],'RowSpacing',5);
    uilabel(cP,'Text','📍 Robot state',...
       'FontWeight','bold','FontSize',15);
    kin_values = uigridlayout(cP,[1,3], ...
        'BackgroundColor',C.card,...
        'ColumnWidth',{'1x', '1x', '1x'},...
        'Padding',[4 0 5 0], ...
        'ColumnSpacing',5);

    xCard = mkDataCard(kin_values);
    [~, lblX]  = mkDataRow(xCard, 'X Position', 'm'); xCard.Layout.Row =1; xCard.Layout.Column =1; 
    yCard = mkDataCard(kin_values);
    [~, lblY]  = mkDataRow(yCard, 'Y Position', 'm'); yCard.Layout.Row =1; yCard.Layout.Column =2;
    thCard = mkDataCard(kin_values);
    [~, lblTh] = mkDataRow(thCard, 'Heading (θ)', '°'); thCard.Layout.Row =1; thCard.Layout.Column =3;

    vel_values = uigridlayout(cP,[1,2], ...
        'BackgroundColor',C.card,...
        'ColumnWidth',{'1x', '1x'},...
        'Padding',[5 0 2 0],...
        'ColumnSpacing',5);
    vCard = mkDataCard(vel_values);
    [~, lblVv] = mkDataRow(vCard, 'Linear velocity (v)', 'm/s'); vCard.Layout.Row =1; vCard.Layout.Column =1;
    omgCard = mkDataCard(vel_values);
    [~, lblomega] = mkDataRow(omgCard, 'Angular velocity (ω)', 'rad/s'); omgCard.Layout.Row =1; omgCard.Layout.Column =2; 
    
    % Card 2: Sensors
    sensorsCard = uigridlayout(sidebar,[2,1], ...
       'BackgroundColor',C.card,...
       'RowHeight',{25, '1x'},...
       'Padding',[10 10 10 10],'RowSpacing',5);
    uilabel(sensorsCard,'Text','📡 Sensors',...
       'FontWeight','bold','FontSize',15);
    sensor_values = uigridlayout(sensorsCard,[1,3], ...
        'BackgroundColor',C.card,...
        'ColumnWidth',{'1x', '1x', '1x'},...
        'Padding',[4 0 5 0], ...
        'ColumnSpacing',5);
    
    leftCard = mkDataCard(sensor_values);
    [~, lblDistL] = mkDataRow(leftCard, 'Left', 'm');  leftCard.Layout.Row = 1; leftCard.Layout.Column = 1;
    frontCard = mkDataCard(sensor_values);
    [~, lblDistF] = mkDataRow(frontCard, 'Front', 'm'); frontCard.Layout.Row = 1; frontCard.Layout.Column = 2;
    rightCard = mkDataCard(sensor_values);
    [~, lblDistR] = mkDataRow(rightCard, 'Right', 'm'); rightCard.Layout.Row = 1; rightCard.Layout.Column = 3;

    % Card 3: Actuators (Wheel Speeds)
  
    actuatorsCard = uigridlayout(sidebar,[4,1], ...
       'BackgroundColor',C.card,...
       'RowHeight',{25, '1x',20,'1x'},...
       'Padding',[10 10 10 10],'RowSpacing',5);
    uilabel(actuatorsCard,'Text','⚙️ ACTUATORS',...
       'FontWeight','bold','FontSize',15);
    gaugeL = uigauge(actuatorsCard,'linear','Limits',[-MAX_V/CELL MAX_V/CELL],'Value',0,...
        'ScaleColors',{C.primary},'ScaleColorLimits',[0 MAX_V/CELL],...
        'BackgroundColor',C.bg,'FontColor',C.textMuted,'FontSize',10);
        uilabel(actuatorsCard,'Text','Left Motor (vL)','FontColor',C.textMuted,'FontName',fontSans,'HorizontalAlignment','center');

    gaugeR = uigauge(actuatorsCard,'linear','Limits',[-MAX_V/CELL MAX_V/CELL],'Value',0,...
        'ScaleColors',{C.primary},'ScaleColorLimits',[0 MAX_V/CELL],...
        'BackgroundColor',C.bg,'FontColor',C.textMuted,'FontSize',10);
    uilabel(actuatorsCard,'Text','Right Motor (vR)','FontColor',C.textMuted,'FontName',fontSans,'HorizontalAlignment','center');

    % Card 4: D-Pad Controls
    controllersCard = uigridlayout(sidebar,[3,3], ...
       'BackgroundColor',C.card,...
       'RowHeight',{25, '1x','1x'},...
       'Padding',[10 10 10 10],'RowSpacing',5);
    uilabel(controllersCard,'Text','🕹️ MANUAL',...
       'FontWeight','bold','FontSize',15);
    kW = mkDpadBtn(controllersCard,'▲'); kW.Layout.Row=2; kW.Layout.Column=2;
    kA = mkDpadBtn(controllersCard,'⭯'); kA.Layout.Row=3; kA.Layout.Column=1;
    kS = mkDpadBtn(controllersCard,'▼'); kS.Layout.Row=3; kS.Layout.Column=2;
    kD = mkDpadBtn(controllersCard,'⭮'); kD.Layout.Row=3; kD.Layout.Column=3;
    if useExternal
        btnMode.BackgroundColor = C.primary; btnMode.FontColor = C.bg;
        kW.Enable = 'off'; kA.Enable = 'off'; kS.Enable = 'off'; kD.Enable = 'off';
    end
    
    
    % --- BOTTOM NAVIGATION BAR ---
    bottomNav = uigridlayout(mainGrid,[1,7],...
        'BackgroundColor',C.card,...
        'ColumnWidth',{150,40,'1x',40,'1x',40,'1x'},...
        'Padding',[24 10 18 10],'ColumnSpacing',10);
    uilabel(bottomNav,'Text','📊 STATISTICS',...
        'FontColor',C.text,'FontWeight','bold','FontSize',15,'FontName',fontSans);
    [~, lblTime] = mkDataRow(bottomNav, 'Uptime', 's'); 
    [~, lblDist] = mkDataRow(bottomNav, 'Distance', 'm');
    [lblStatTitle, lblStat] = mkDataRow(bottomNav, 'Status', '');
    lblStat.Text = 'ONLINE'; lblStat.FontColor = C.primary;
    %% ── 4. Graphics Objects ─────────────────────────────────────────────────
    hImg   = image(ax,'CData',zeros(15,15,3),'XData',[0 15],'YData',[0 15]);
    hTrail = plot(ax, trailX, trailY,'-','Color',[C.robot 0.5],'LineWidth',2.0);
    
    % Robot Patches
    hBody  = patch(ax,'XData',0,'YData',0,'FaceColor',C.robot, 'EdgeColor','none');
    hWL    = patch(ax,'XData',0,'YData',0,'FaceColor',C.wheel, 'EdgeColor','none');
    hWR    = patch(ax,'XData',0,'YData',0,'FaceColor',C.wheel, 'EdgeColor','none');
    hFwd   = patch(ax,'XData',0,'YData',0,'FaceColor',C.bg, 'EdgeColor','none','FaceAlpha',0.8);
    
    hStart = plot(ax,0,0,'o','Color',C.primary,'MarkerSize',14,'LineWidth',3);
    hGoalC = plot(ax,0,0,'o','Color',C.warning,'MarkerSize',16,'LineWidth',2);
    hGoalX = plot(ax,0,0,'x','Color',C.warning,'MarkerSize',10,'LineWidth',4);

    %% ── 5. Logic: Maze & Init ───────────────────────────────────────────────
    function grid = genMaze(cols, rows)
        grid = ones(rows, cols); visited = false(rows, cols);
        stack = [2, 2]; visited(2,2) = true; grid(2,2) = 0;
        DIRS = [0 2; 0 -2; 2 0; -2 0];
        while ~isempty(stack)
            r = stack(end,1); c = stack(end,2); d = DIRS(randperm(4),:); moved = false;
            for i = 1:4
                nr = r+d(i,1); nc = c+d(i,2);
                if nr>=1 && nr<=rows && nc>=1 && nc<=cols && ~visited(nr,nc)
                    grid(r+d(i,1)/2, c+d(i,2)/2) = 0; grid(nr,nc) = 0;
                    visited(nr,nc) = true; stack(end+1,:) = [nr, nc]; %#ok<AGROW>
                    moved = true; break;
                end
            end
            if ~moved, stack(end,:) = []; end
        end
        grid(rows-1, cols-1) = 0;
    end

    function initSim(newMaze)
        N = SIZES(sizeIdx); btnSz.Text = sprintf('📐 Size: %dx%d', N, N);
        if newMaze, maze = genMaze(N, N); end
        
        goalPos.x = (N - 1.5) * CELL; goalPos.y = (N - 1.5) * CELL;

        % Build the image tensor
        img = zeros(N, N, 3);
        for ch = 1:3
            img(:,:,ch) = maze*C.wall(ch) + (1-maze)*C.passage(ch);
        end
        
        hImg.CData = img;
        hImg.XData = [CELL/2, N*CELL - CELL/2]; hImg.YData = [CELL/2, N*CELL - CELL/2];
        
        ax.XLim = [-0.3*CELL, N*CELL + 0.3*CELL]; ax.YLim = [-0.3*CELL, N*CELL + 0.3*CELL];
        ax.XLimMode = 'manual'; ax.YLimMode = 'manual'; ax.DataAspectRatioMode = 'manual';

        hStart.XData = startPos.x; hStart.YData = startPos.y;
        hGoalC.XData = goalPos.x;  hGoalC.YData = goalPos.y;
        hGoalX.XData = goalPos.x;  hGoalX.YData = goalPos.y;

        resetRobot();
    end

    function resetRobot()
        robot.x = startPos.x; robot.y = startPos.y;
        robot.theta = 0; robot.v = 0; robot.omega = 0;
        goalReached = false; elapsedTime = 0; totalDist = 0;
        prevX = robot.x; prevY = robot.y;
        trailX(:) = NaN; trailY(:) = NaN; trailPtr = 0;
        hTrail.XData = trailX; hTrail.YData = trailY;
        hGoalC.Color = C.warning; hGoalX.Color = C.warning;
        lblStat.Text = 'ONLINE'; lblStat.FontColor = C.primary;
        lblStatTitle.FontColor = C.textMuted;
    end

    %% ── 6. Logic: Physics & Sensors ─────────────────────────────────────────
    function [vL, vR] = defaultAutoController(~, sensors)
         vL = 0; vR = 0; 
    end

    function dist = getSensorDistance(startX, startY, globalAngle, maxDist)
        step = 0.05 * CELL; dist = 0; currX = startX; currY = startY;
        while dist < maxDist
            currX = currX + cos(globalAngle) * step; currY = currY + sin(globalAngle) * step;
            dist = dist + step;
            c = floor(currX / CELL) + 1; r = floor(currY / CELL) + 1;
            if r < 1 || r > size(maze,1) || c < 1 || c > size(maze,2) || maze(r,c) == 1
                break;
            end
        end
    end

    function hit = hasCollision(nx, ny, th)
        hl = Lr*0.44; hw = Wr*0.44; cns = [hl hw; hl -hw; -hl hw; -hl -hw];
        ct = cos(th); st = sin(th); hit = false;
        for k = 1:4
            px = nx + cns(k,1)*ct - cns(k,2)*st; py = ny + cns(k,1)*st + cns(k,2)*ct;
            c = floor(px / CELL) + 1; r = floor(py / CELL) + 1;
            if r<1 || r>size(maze,1) || c<1 || c>size(maze,2) || maze(r,c)==1
                hit = true; return;
            end
        end
    end

    function updatePhysics(dt, distF, distL, distR)
        if useExternal
            try
                [cmd_vL, cmd_vR] = controllerFcn(struct('x', robot.x/CELL, 'y', robot.y/CELL, ...
                           'theta', robot.theta, 'v', robot.v/CELL, 'omega', robot.omega,'goalReached',goalReached), ...
                           struct('front', distF, 'left', distL, 'right', distR));
            catch, cmd_vL = 0; cmd_vR = 0; end
            
            t_v = (cmd_vL*CELL + cmd_vR*CELL) / 2; t_om = (cmd_vR*CELL - cmd_vL*CELL) / (2 * HWB);
            robot.v = robot.v + sign(t_v - robot.v) * min(abs(t_v - robot.v), ACCEL*dt);
            robot.omega = robot.omega + sign(t_om - robot.omega) * min(abs(t_om - robot.omega), RACCEL*dt);
        else
            fwd = K.w || K.up; bwd = K.s || K.dn; lft = K.a || K.lt; rgt = K.d || K.rt;
            if fwd, robot.v = min(robot.v + ACCEL*dt, MAX_V);
            elseif bwd, robot.v = max(robot.v - ACCEL*dt, -MAX_V*0.55);
            else, robot.v = robot.v * max(0, 1 - LIN_DAMP*dt); end

            if lft, robot.omega = min(robot.omega + RACCEL*dt, MAX_OM);
            elseif rgt, robot.omega = max(robot.omega - RACCEL*dt, -MAX_OM);
            else, robot.omega = robot.omega * max(0, 1 - ANG_DAMP*dt); end
        end

        avgTh = robot.theta + robot.omega * dt * 0.5;
        nx = robot.x + robot.v * cos(avgTh) * dt; ny = robot.y + robot.v * sin(avgTh) * dt;
        newTh = robot.theta + robot.omega * dt;

        if ~hasCollision(nx, ny, newTh)
            robot.x = nx; robot.y = ny; robot.theta = newTh;
        else
            if ~hasCollision(nx, robot.y, newTh), robot.x = nx; robot.theta = newTh; robot.v = robot.v * 0.6;
            elseif ~hasCollision(robot.x, ny, newTh), robot.y = ny; robot.theta = newTh; robot.v = robot.v * 0.6;
            else, robot.v = robot.v * -0.25; robot.omega = robot.omega * 0.40; end
        end

        d = hypot(robot.x - prevX, robot.y - prevY);
        if ~goalReached && (d > 0.06 * CELL)
            totalDist = totalDist + d; prevX = robot.x; prevY = robot.y;
            trailPtr = mod(trailPtr, TRAIL_MAX) + 1;
            trailX(trailPtr) = robot.x; trailY(trailPtr) = robot.y;
            if showTrail, hTrail.XData = trailX; hTrail.YData = trailY; end
        end

        if ~goalReached, elapsedTime = elapsedTime + dt; end
        if ~goalReached && hypot(robot.x-goalPos.x, robot.y-goalPos.y) < 0.8 * CELL
            goalReached = true;
            hGoalC.Color = C.primary; hGoalX.Color = C.primary;
            lblStat.Text = 'GOAL REACHED 👌';
        end
    end

    function drawRobot()
        ct = cos(robot.theta); st = sin(robot.theta); R = [ct,-st; st, ct];
        bx = Lr/2*[-1,1,1,-1]; by = Wr/2*[-1,-1,1,1];
        wLy = HWB + Wr*0.12*[-1,-1,1,1]; wRy = -HWB + Wr*0.12*[-1,-1,1,1]; wx = Lr*0.65/2*[-1,1,1,-1];
        fx = [Lr*0.15, Lr*0.15, Lr*0.35]; fy = [-Wr*0.2, Wr*0.2, 0];
        
        function [rx,ry] = xf(lx,ly), p = R*[lx;ly]; rx = p(1,:)+robot.x; ry = p(2,:)+robot.y; end

        [bwx,bwy] = xf(bx, by); [wlx,wly] = xf(wx, wLy); [wrx,wry] = xf(wx, wRy); [fwx,fwy] = xf(fx, fy);
        hBody.XData = bwx; hBody.YData = bwy; hWL.XData = wlx; hWL.YData = wly;
        hWR.XData = wrx; hWR.YData = wry; hFwd.XData = fwx; hFwd.YData = fwy;
    end

    %% ── 7. UI Interaction Logic ─────────────────────────────────────────────
    function toggleMode()
        useExternal = ~useExternal;
        initSim(true);
        if useExternal
            btnMode.Text = '🤖 MODE: AUTO';
            btnMode.BackgroundColor = C.primary; btnMode.FontColor = C.bg;
            % Fade D-PAD
            kW.Enable = 'off'; kA.Enable = 'off'; kS.Enable = 'off'; kD.Enable = 'off';
            K.w=false; K.s=false; K.a=false; K.d=false;
        else
            btnMode.Text = '🎮 MODE: MANUAL';
            btnMode.BackgroundColor = C.cardHover; btnMode.FontColor = C.text;
            % Activate D-PAD
            kW.Enable = 'on'; kA.Enable = 'on'; kS.Enable = 'on'; kD.Enable = 'on';
        end
    end

    function onKey(key, pressed)
        if useExternal, return; end
        switch lower(key)
            case {'w','uparrow'},   K.w = pressed; setKey(kW, pressed);
            case {'s','downarrow'}, K.s = pressed; setKey(kS, pressed);
            case {'a','leftarrow'}, K.a = pressed; setKey(kA, pressed);
            case {'d','rightarrow'},K.d = pressed; setKey(kD, pressed);
            case 'r', if pressed, resetRobot(); end
        end
    end

    function setKey(btn, pressed)
        if pressed, btn.BackgroundColor = C.primaryDim; btn.FontColor = C.text;
        else,       btn.BackgroundColor = C.bg; btn.FontColor = C.textMuted; end
    end

    function changeSize(), sizeIdx = mod(sizeIdx, numel(SIZES)) + 1; initSim(true); end
    function toggleTrail()
        showTrail = ~showTrail;
        if showTrail
            hTrail.Visible = 'on';
            btnTrail.Text  = '👣 Trail ON';
            btnTrail.FontColor = C.text;
            btnTrail.BackgroundColor = C.primaryDim;
        else
            hTrail.Visible = 'off';
            btnTrail.Text  = '👣 Trail OFF';
            btnTrail.FontColor = C.text;
            btnTrail.BackgroundColor = C.cardHover;
        end
    end
    %% ── 8. UI Factory Helpers ───────────────────────────────────────────────
    % Fixed: Removed BorderColor

    function g = mkDataCard(parent)
        g = uigridlayout(parent, [2,1], 'BackgroundColor',C.passage,'Padding',[8 0 0 0],'RowSpacing',0);
    end
    function [lblTitle, lblVal] = mkDataRow(parent, title, unit)
        if ~isempty(unit), valTxt = ['0.00 ', unit]; else, valTxt = '---'; end
        lblTitle = uilabel(parent,'Text',title,'FontColor',C.textMuted,'FontName',fontSans,'FontSize',10);
        
        lblVal = uilabel(parent,'Text',valTxt,'FontColor',C.primary,'FontWeight','bold',...
            'FontName',fontMono,'FontSize',25,'HorizontalAlignment','left');
        
    end

    % Fixed: Removed BorderColor
    function btn = mkDpadBtn(parent, txt)
        btn = uibutton(parent,'Text',txt,'BackgroundColor',C.bg,'FontColor',C.textMuted,...
            'FontName',fontSans,'FontSize',30,'FontWeight','bold');
    end

    %% ── 9. Main Loop ────────────────────────────────────────────────────────
    initSim(true); t = tic;
    
    while ishandle(fig)
        dt = min(toc(t), 0.05); t = tic;

        % Sense
        distF = getSensorDistance(robot.x, robot.y, robot.theta, 5.0*CELL);
        distL = getSensorDistance(robot.x, robot.y, robot.theta - pi/2, 5.0*CELL);
        distR = getSensorDistance(robot.x, robot.y, robot.theta + pi/2, 5.0*CELL);
        
        actF = max(0, (distF - Lr/2) / CELL);
        actL = max(0, (distL - Wr/2) / CELL);
        actR = max(0, (distR - Wr/2) / CELL);

        % Think & Act
        updatePhysics(dt, actF, actL, actR);
        drawRobot();

        % UI Update
        lblX.Text  = sprintf('%.2f m', robot.x / CELL); 
        lblY.Text  = sprintf('%.2f m', robot.y / CELL);
        lblTh.Text = sprintf('%.1f°', mod((robot.theta*180/pi), 360));
        lblVv.Text = sprintf('%.2f m/s', abs(robot.v) / CELL);
        lblomega.Text = sprintf('%.2f rad/s', abs(robot.omega));

        % Sensor text color change on proximity warning
        if actF < 0.6, lblDistF.FontColor = C.warning; else, lblDistF.FontColor = C.primary; end
        if actL < 0.4, lblDistL.FontColor = C.warning; else, lblDistL.FontColor = C.primary; end
        if actR < 0.4, lblDistR.FontColor = C.warning; else, lblDistR.FontColor = C.primary; end

        lblDistF.Text = sprintf('%.2f m', actF);
        lblDistL.Text = sprintf('%.2f m', actL);
        lblDistR.Text = sprintf('%.2f m', actR);
        
        vL_disp = (robot.v - robot.omega * HWB) / CELL;
        vR_disp = (robot.v + robot.omega * HWB) / CELL;
        gaugeL.Value = max(-MAX_V/CELL, min(MAX_V/CELL, vL_disp));
        gaugeR.Value = max(-MAX_V/CELL, min(MAX_V/CELL, vR_disp));

        lblTime.Text = sprintf('%.1f s', elapsedTime);
        lblDist.Text = sprintf('%.2f m', totalDist / CELL);

        drawnow limitrate; 
    end
end
