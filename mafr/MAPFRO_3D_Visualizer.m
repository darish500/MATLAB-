function MAPFRO_3D_Visualizer(varargin)
% MAPFRO_3D_VISUALIZER - Real-time 3D MAPF-RO simulation
%
% ▶ Play       → build + plan + animate (only button that runs animation)
% 🔀 New Env   → rebuild world, place robots at start, WAIT for Play
% ➕/➖ Robots  → change count, rebuild world, WAIT for Play
% Camera buttons → orbit view only
%
% Usage:
%   MAPFRO_3D_Visualizer()
%   MAPFRO_3D_Visualizer('nRobots',8,'size','medium')
%   MAPFRO_3D_Visualizer(simObj)

%% ── Parse input ──────────────────────────────────────────────────────
if nargin==1 && isa(varargin{1},'Simulation')
    initSim  = varargin{1};
    initNR   = numel(initSim.robots);
    initSize = initSim.gridSize;
    initK    = initSim.pushK;
else
    p = inputParser;
    addParameter(p,'nRobots', 5,      @isnumeric);
    addParameter(p,'size',   'small', @ischar);
    addParameter(p,'k',       2,      @isnumeric);
    addParameter(p,'seed',    0,      @isnumeric);
    parse(p, varargin{:});
    initNR   = p.Results.nRobots;
    initSize = p.Results.size;
    initK    = p.Results.k;
    initSim  = [];
end

PAL = robotPalette();

%% ── Figure ───────────────────────────────────────────────────────────
fig = uifigure('Name','MAPF-RO  |  Gazebo 3D', ...
    'Position',[20 20 1440 840], ...
    'Color',[0.06 0.06 0.08]);

%% Info strip (top)
infoLbl = uilabel(fig, ...
    'Position',[0 808 1440 32], ...
    'FontSize',10.5,'FontWeight','bold', ...
    'FontColor',[0.75 0.88 1.00], ...
    'BackgroundColor',[0.05 0.07 0.15], ...
    'HorizontalAlignment','left', ...
    'Text','  Press ▶ Play to start');

%% 3D axes (left 75%) — NO axis lines, ticks, labels, grid numbers
ax = axes(fig,'Position',[0.005 0.09 0.755 0.89]);
ax.Color            = [0.05 0.05 0.06];
ax.XColor           = 'none';   % hide X axis completely
ax.YColor           = 'none';   % hide Y axis completely
ax.ZColor           = 'none';   % hide Z axis completely
ax.XGrid            = 'off';
ax.YGrid            = 'off';
ax.ZGrid            = 'off';
ax.Box              = 'off';
ax.XAxis.Visible    = 'off';
ax.YAxis.Visible    = 'off';
ax.ZAxis.Visible    = 'off';
ax.DataAspectRatio  = [1 1 1];
hold(ax,'on');
view(ax,[-38,52]);
camproj(ax,'perspective');

%% Right panel
rp    = uipanel(fig,'Position',[1094 95 340 715], ...
    'BackgroundColor',[0.08 0.08 0.10],'BorderType','none');
axBar = uiaxes(rp,'Position',[5 380 330 325]);
axBar.Color=[0.10 0.10 0.12];
axBar.XColor=[0.60 0.65 0.72]; axBar.YColor=[0.60 0.65 0.72];
axBar.YGrid='on'; axBar.GridColor=[0.22 0.25 0.30];
title(axBar,'Energy per Robot','Color',[0.85 0.90 1.0],'FontSize',9);

axInfo = uiaxes(rp,'Position',[5 5 330 365]);
axInfo.Color=[0.08 0.08 0.10];
axInfo.XAxis.Visible='off'; axInfo.YAxis.Visible='off';
axInfo.XLim=[0 1]; axInfo.YLim=[0 1];

%% Status bar
statusLbl = uilabel(fig,'Text','  Ready', ...
    'Position',[1094 62 340 28], ...
    'FontSize',9,'FontColor',[0.50 0.80 0.50], ...
    'BackgroundColor',[0.07 0.08 0.10]);

%% ── Shared state ─────────────────────────────────────────────────────
state.nR      = initNR;
state.size    = initSize;
state.k       = initK;
state.busy    = false;
state.envSeed = 0;
state.builtSim    = [];
state.builtRobotH = {};
state.builtTrailH = {};
state.builtLabelH = {};
state.builtLidarH = {};
state.pathHandles = gobjects(0,1);
state.goalHandles = gobjects(0,1);

%% ── Control bar ──────────────────────────────────────────────────────
btnH=48; y0=10; x=8;

btnPlay = uibutton(fig,'push','Text','▶  Play', ...
    'Position',[x y0 120 btnH], ...
    'BackgroundColor',[0.10 0.45 0.18], ...
    'FontColor',[1 1 1],'FontSize',13,'FontWeight','bold', ...
    'ButtonPushedFcn',@(~,~) onPlay());
x=x+124;

cams={'⬆ Top','◆ Iso','◀ Side','▶ Front'};
cazim=[0,-38,-90,0]; celev=[90,52,0,0];
for bi=1:4
    az=cazim(bi); el=celev(bi);
    uibutton(fig,'push','Text',cams{bi}, ...
        'Position',[x y0 76 btnH], ...
        'BackgroundColor',[0.13 0.14 0.24], ...
        'FontColor',[0.78 0.84 1.00],'FontSize',9, ...
        'ButtonPushedFcn',@(~,~) view(ax,[az,el]));
    x=x+80;
end
x=x+6;

uibutton(fig,'push','Text','🔀  New Env', ...
    'Position',[x y0 118 btnH], ...
    'BackgroundColor',[0.30 0.18 0.06], ...
    'FontColor',[1.00 0.84 0.50],'FontSize',10,'FontWeight','bold', ...
    'ButtonPushedFcn',@(~,~) onNew('same'));
x=x+122;

uibutton(fig,'push','Text','➕ Robots', ...
    'Position',[x y0 100 btnH], ...
    'BackgroundColor',[0.10 0.28 0.12], ...
    'FontColor',[0.65 1.00 0.65],'FontSize',10,'FontWeight','bold', ...
    'ButtonPushedFcn',@(~,~) onNew('more'));
x=x+104;

uibutton(fig,'push','Text','➖ Robots', ...
    'Position',[x y0 100 btnH], ...
    'BackgroundColor',[0.28 0.08 0.08], ...
    'FontColor',[1.00 0.65 0.65],'FontSize',10,'FontWeight','bold', ...
    'ButtonPushedFcn',@(~,~) onNew('less'));
x=x+104;

lblRobots = uilabel(fig,'Text',sprintf('n=%d',initNR), ...
    'Position',[x+4 y0+10 58 26], ...
    'FontSize',11,'FontWeight','bold', ...
    'FontColor',[0.65 0.95 0.65], ...
    'BackgroundColor',[0.08 0.10 0.08], ...
    'HorizontalAlignment','center');

%% ── Initial world build (no animation) ──────────────────────────────
if ~isempty(initSim)
    buildAndShow(initSim);
    setStatus('Environment loaded — press ▶ Play to animate',[0.60 0.80 1.00]);
else
    onNew('same');   % build initial world silently
end

%% ══════════════════════════════════════════════════════════════════════
%% CALLBACKS
%% ══════════════════════════════════════════════════════════════════════

    %% ▶ Play — only button that runs the animation ─────────────────────
    function onPlay()
        if state.busy, return; end
        if isempty(state.builtSim)
            setStatus('Building environment first...',[0.95 0.82 0.22]);
            drawnow;
            buildWorld(state.nR, state.size, state.k);
            if isempty(state.builtSim), return; end
        end
        state.busy = true;
        btnPlay.Enable = 'off';
        runAnimation();
        btnPlay.Enable = 'on';
        state.busy = false;
    end

    %% 🔀 New Env — full rebuild with new random seed ────────────────────
    function onNew(mode)
        if state.busy, return; end
        switch mode
            case 'same'
                % Full new random environment
                buildWorld(state.nR, state.size, state.k);

            case 'more'
                % Add robots to EXISTING world — do NOT clear scene
                state.nR = min(state.nR + 1, 20);
                lblRobots.Text = sprintf('n=%d', state.nR);
                if isempty(state.builtSim)
                    buildWorld(state.nR, state.size, state.k);
                else
                    rebuildRobotsOnly(state.nR);
                end

            case 'less'
                % Remove robots from EXISTING world — do NOT clear scene
                state.nR = max(state.nR - 1, 1);
                lblRobots.Text = sprintf('n=%d', state.nR);
                if isempty(state.builtSim)
                    buildWorld(state.nR, state.size, state.k);
                else
                    rebuildRobotsOnly(state.nR);
                end
        end
    end

    %% Rebuild only robots on the SAME environment ────────────────────────
    function rebuildRobotsOnly(newNR)
        if ~isvalid(fig), return; end
        oldSim = state.builtSim;
        env    = oldSim.env;
        R      = env.rows;

        setStatus(sprintf('Adjusting to %d robots on same map — replanning...', newNR), ...
            [0.95 0.82 0.22]); drawnow;

        % Delete old robot graphics
        for i = 1:numel(state.builtRobotH)
            if ~isempty(state.builtRobotH{i})
                delete(state.builtRobotH{i});
            end
        end
        for i = 1:numel(state.builtLidarH)
            if ~isempty(state.builtLidarH{i})
                delete(state.builtLidarH{i});
            end
        end
        for i = 1:numel(state.builtTrailH)
            if ishandle(state.builtTrailH{i})
                delete(state.builtTrailH{i});
            end
        end
        for i = 1:numel(state.builtLabelH)
            if ishandle(state.builtLabelH{i})
                delete(state.builtLabelH{i});
            end
        end

        % Remove old path lines and goal markers (tagged handles)
        if isfield(state,'pathHandles')
            for i=1:numel(state.pathHandles)
                if ishandle(state.pathHandles(i)), delete(state.pathHandles(i)); end
            end
        end
        if isfield(state,'goalHandles')
            for i=1:numel(state.goalHandles)
                if ishandle(state.goalHandles(i)), delete(state.goalHandles(i)); end
            end
        end

        % Build new sim using SAME grid — just change nRobots
        % We rebuild sim but reuse the environment grid by seeding the
        % same rng state so the obstacles stay identical
        try
            rng(state.envSeed);   % replay same obstacle layout
            newSim = Simulation('size', state.size, 'nRobots', newNR, ...
                'k', state.k, 'animate', false, 'useRO', true);
            newSim.initialise();
            newSim.plan();
        catch e
            setStatus(['Replan error: ' e.message],[1 0.4 0.4]); return;
        end

        robots = newSim.robots;
        R      = newSim.env.rows;

        % Draw new goal markers and paths
        gh = drawGoalMarkersTagged(ax, robots, R, PAL, newNR);
        ph = drawPathLinesTagged(ax, newSim.finalPaths, R, PAL, newNR);
        state.goalHandles = gh;
        state.pathHandles = ph;

        % Draw robots at start
        hR = cell(newNR,1); hL = cell(newNR,1);
        hT = cell(newNR,1); hLbl = cell(newNR,1);
        for i=1:newNR
            s = robots{i}.startPos;
            wx=s(2)-0.5; wy=R-s(1)+0.5;
            col=PAL(mod(i-1,size(PAL,1))+1,:);
            [hR{i},hL{i}] = drawCar(ax,wx,wy,col,0);
            hLbl{i} = text(ax,wx,wy,1.90,sprintf('R%d',i), ...
                'Color','w','FontSize',14,'FontWeight','bold', ...
                'HorizontalAlignment','center');
            hT{i} = plot3(ax,wx,wy,0.08,'-','Color',[col,0.55],'LineWidth',2.0);
        end

        state.builtSim    = newSim;
        state.builtRobotH = hR;
        state.builtLidarH = hL;
        state.builtTrailH = hT;
        state.builtLabelH = hLbl;

        m0=struct('totalEnergy',0,'successCount',0,'nRobots',newNR, ...
            'avgEnergy',0,'removals',numel(newSim.removalPlan),'steps',0,'successRate',0);
        liveUpdateBars(robots,newNR);
        liveUpdateMetrics(m0,newNR);
        hideAxesDecorations();
        setStatus(sprintf('Ready — %d robots on same map. Press ▶ Play', newNR),[0.60 0.80 1.00]);
        drawnow;
    end

%% ══════════════════════════════════════════════════════════════════════
%% BUILD WORLD  (plan paths, draw scene, place robots — no movement)
%% ══════════════════════════════════════════════════════════════════════

    function buildWorld(nR, gridSz, k)
        if ~isvalid(fig), return; end
        setStatus(sprintf('Building %s env — %d robots...', upper(gridSz), nR), ...
            [0.95 0.82 0.22]);
        drawnow;
        % Save seed so ➕/➖ can replay same obstacle layout
        rng('shuffle');
        state.envSeed = rng().Seed;
        rng(state.envSeed);
        try
            sim = Simulation('size',gridSz,'nRobots',nR,'k',k, ...
                'animate',false,'useRO',true);
            sim.initialise();
        catch e
            setStatus(['Build error: ' e.message],[1 0.4 0.4]);
            return;
        end
        setStatus('Planning paths...',[0.65 0.75 0.90]); drawnow;
        try
            sim.plan();
        catch e
            setStatus(['Plan error: ' e.message],[1 0.4 0.4]);
            return;
        end
        buildAndShow(sim);
        setStatus(sprintf('Ready — %d robots, %d removals planned. Press ▶ Play', ...
            nR, numel(sim.removalPlan)), [0.60 0.80 1.00]);
    end

    function buildAndShow(sim)
        % Draw static world + robots at start positions. Store handles.
        env    = sim.env;
        robots = sim.robots;
        nR     = numel(robots);
        R      = env.rows; C = env.cols;

        clearScene();
        setupLights(R,C);
        drawWarehouseFloor(ax,R,C);
        drawPerimeterWalls(ax,R,C);
        drawStaticObstacles(ax,env,R);
        drawPits(ax,env,R);
        drawSandbags(ax,env,R);
        drawGoalMarkers(ax,robots,R,PAL,nR);
        drawPathLines(ax,sim.finalPaths,R,PAL,nR);
        drawAssignmentArrows(ax,sim.removalPlan,R);

        % Place robots at START
        hR = cell(nR,1); hL = cell(nR,1);
        hT = cell(nR,1); hLbl = cell(nR,1);
        for i=1:nR
            s = robots{i}.startPos;
            wx=s(2)-0.5; wy=R-s(1)+0.5;
            col=PAL(mod(i-1,size(PAL,1))+1,:);
            [hR{i},hL{i}] = drawCar(ax,wx,wy,col,0);
            hLbl{i} = text(ax,wx,wy,1.90,sprintf('R%d',i), ...
                'Color','w','FontSize',14,'FontWeight','bold', ...
                'HorizontalAlignment','center');
            hT{i} = plot3(ax,wx,wy,0.08,'-','Color',[col,0.55],'LineWidth',2.0);
        end
        state.builtSim    = sim;
        state.builtRobotH = hR;
        state.builtLidarH = hL;
        state.builtTrailH = hT;
        state.builtLabelH = hLbl;

        axis(ax,'tight');
        zlim(ax,[-0.25, max(4.0, R*0.20)]);
        hideAxesDecorations();
        setTitle(sim);

        % Zero energy panel
        m0=struct('totalEnergy',0,'successCount',0,'nRobots',nR, ...
            'avgEnergy',0,'removals',numel(sim.removalPlan),'steps',0,'successRate',0);
        liveUpdateBars(robots,nR);
        liveUpdateMetrics(m0,nR);
        infoLbl.Text = makeInfoText(env,nR,sim.removalPlan,m0);
        drawnow;
    end

%% ══════════════════════════════════════════════════════════════════════
%% ANIMATION  (called only by onPlay)
%% ══════════════════════════════════════════════════════════════════════

    function runAnimation()
        if isempty(state.builtSim) || ~isvalid(fig), return; end

        sim    = state.builtSim;
        env    = sim.env;
        robots = sim.robots;
        nR     = numel(robots);
        R      = env.rows;
        paths  = sim.finalPaths;
        plan   = sim.removalPlan;

        % Reuse the handles already placed by buildAndShow
        hRobots  = state.builtRobotH;
        hLidars  = state.builtLidarH;
        hTrails  = state.builtTrailH;
        hLabels  = state.builtLabelH;
        lidarAng = zeros(nR,1);

        SUBSTEPS = 3;
        maxSteps = R*env.cols*3;
        stepCount = 0;

        trailX=cell(nR,1); trailY=cell(nR,1); trailZ=cell(nR,1);
        prevWX=zeros(nR,1); prevWY=zeros(nR,1);
        for i=1:nR
            s=robots{i}.startPos;
            prevWX(i)=s(2)-0.5; prevWY(i)=R-s(1)+0.5;
            trailX{i}=prevWX(i); trailY{i}=prevWY(i); trailZ{i}=0.08;
        end

        setStatus(sprintf('Animating — %d robots | press camera buttons to orbit', nR), ...
            [0.95 0.88 0.25]);

        waitCounts = zeros(nR,1);  % track how long each robot has waited

        while stepCount < maxSteps
            if ~isvalid(fig), return; end
            allDone = all(cellfun(@(r) r.isFinished(), robots));
            if allDone, break; end

            % One logical step
            occupied = zeros(nR,2);
            for i=1:nR, occupied(i,:)=robots{i}.currentPos; end

            for i=1:nR
                r=robots{i};
                if r.isFinished(), continue; end
                np=getNextPos(r);
                blocked = false;
                if ~isempty(np)
                    oth=occupied; oth(i,:)=[];
                    if any(oth(:,1)==np(1) & oth(:,2)==np(2))
                        blocked = true;
                    end
                end

                if blocked
                    r.pause();
                    waitCounts(i) = waitCounts(i) + 1;
                    % Deadlock break: if waiting >6 steps, replan with
                    % current blocker cell marked as occupied
                    if waitCounts(i) > 6
                        oth2 = occupied; oth2(i,:) = [];
                        [newPath,~] = sim.planner.plan( ...
                            r.currentPos, r.goalPos, ...
                            'occupiedCells', oth2);
                        if ~isempty(newPath)
                            r.setPath(newPath);
                        end
                        waitCounts(i) = 0;
                    end
                else
                    r.resume();
                    waitCounts(i) = 0;
                end
                sim.executeRobotStep(r,i);
            end
            stepCount=stepCount+1;

            % World coords after step
            destWX=zeros(nR,1); destWY=zeros(nR,1);
            for i=1:nR
                cp=robots{i}.currentPos;
                destWX(i)=cp(2)-0.5; destWY(i)=R-cp(1)+0.5;
            end

            % Smooth sub-steps
            for ss=1:SUBSTEPS
                if ~isvalid(fig), return; end
                t=ss/SUBSTEPS;
                for i=1:nR
                    wx=prevWX(i)*(1-t)+destWX(i)*t;
                    wy=prevWY(i)*(1-t)+destWY(i)*t;
                    col=PAL(mod(i-1,size(PAL,1))+1,:);
                    lidarAng(i)=lidarAng(i)+14;
                    delete(hRobots{i}); delete(hLidars{i});
                    [hRobots{i},hLidars{i}]=drawCar(ax,wx,wy,col,lidarAng(i));
                    hLabels{i}.Position=[wx,wy,1.90];
                end
                drawnow limitrate;
                pause(0.005);
            end

            % Update trails
            for i=1:nR
                trailX{i}(end+1)=destWX(i);
                trailY{i}(end+1)=destWY(i);
                trailZ{i}(end+1)=0.08;
                set(hTrails{i},'XData',trailX{i},'YData',trailY{i},'ZData',trailZ{i});
            end
            prevWX=destWX; prevWY=destWY;

            % Live charts every 4 steps
            if mod(stepCount,4)==0
                mL.totalEnergy  = sum(cellfun(@(r) r.energyTotal, robots));
                mL.avgEnergy    = mean(cellfun(@(r) r.energyTotal, robots));
                mL.successCount = sum(cellfun(@(r) r.isDone(), robots));
                mL.successRate  = mL.successCount/nR;
                mL.nRobots=nR; mL.removals=numel(plan); mL.steps=stepCount;
                liveUpdateBars(robots,nR);
                liveUpdateMetrics(mL,nR);
                setStatus(sprintf('Step %d  |  Done %d/%d  |  Energy %.0f', ...
                    stepCount,mL.successCount,nR,mL.totalEnergy),[0.60 0.85 0.60]);
            end
        end

        % Final
        sim.collectMetrics();
        m=sim.metrics; m.steps=stepCount;
        liveUpdateBars(robots,nR);
        liveUpdateMetrics(m,nR);
        infoLbl.Text=makeInfoText(env,nR,plan,m);
        setTitle(sim);
        setStatus(sprintf('✓ Done!  %d/%d reached goals  |  Energy=%.0f  |  %d removals', ...
            m.successCount,nR,m.totalEnergy,m.removals),[0.25 0.95 0.42]);

        % Keep sim so user can re-play
        state.builtSim=sim;
        state.builtRobotH=hRobots;
        state.builtLidarH=hLidars;
        state.builtTrailH=hTrails;
        state.builtLabelH=hLabels;
    end

%% ── Helpers ──────────────────────────────────────────────────────────

    function clearScene()
        cla(ax); hold(ax,'on');
    end

    function setupLights(R,C)
        light(ax,'Position',[ C*0.5,  R*0.5,  R*2.5],'Style','infinite','Color',[1.00 0.95 0.85]);
        light(ax,'Position',[-C*0.8,  R*0.2,  R*0.8],'Style','infinite','Color',[0.20 0.28 0.55]);
        light(ax,'Position',[ C*0.3, -R*0.8,  R*0.4],'Style','infinite','Color',[0.35 0.22 0.45]);
        lighting(ax,'gouraud');
        material(ax,'dull');
    end

    function hideAxesDecorations()
        ax.XAxis.Visible='off'; ax.YAxis.Visible='off'; ax.ZAxis.Visible='off';
        ax.XGrid='off'; ax.YGrid='off'; ax.ZGrid='off';
        ax.Box='off';
    end

    function setTitle(sim)
        nRob = numel(sim.robots);
        R2   = sim.env.rows; C2 = sim.env.cols;
        m2   = sim.metrics;
        if ~isfield(m2,'totalEnergy') || m2.totalEnergy == 0
            tstr = sprintf('MAPF-RO  |  %dx%d  |  %d Robots  |  %d Removals planned', ...
                R2, C2, nRob, numel(sim.removalPlan));
        else
            tstr = sprintf('MAPF-RO  |  %dx%d  |  %d/%d Success  |  Energy=%.0f', ...
                R2, C2, m2.successCount, nRob, m2.totalEnergy);
        end
        title(ax, tstr, 'Color',[0.85 0.92 1.00],'FontSize',11,'FontWeight','bold');
    end

    function setStatus(msg,col)
        if isvalid(statusLbl)
            statusLbl.Text=['  ' msg];
            statusLbl.FontColor=col;
        end
    end

    function liveUpdateBars(robs,n)
        if ~isvalid(axBar), return; end
        moveE=cellfun(@(r) r.energyMove, robs);
        pushE=cellfun(@(r) r.energyPush, robs);
        cla(axBar);
        b=bar(axBar,[moveE;pushE]','stacked','EdgeColor','none');
        b(1).FaceColor=[0.22 0.55 1.00];
        if numel(b)>1, b(2).FaceColor=[1.00 0.42 0.08]; end
        axBar.XTickLabel=arrayfun(@(i) sprintf('R%d',i),1:n,'UniformOutput',false);
        axBar.Color=[0.10 0.10 0.12];
        axBar.XColor=[0.60 0.65 0.72]; axBar.YColor=[0.60 0.65 0.72];
        axBar.YGrid='on'; axBar.GridColor=[0.22 0.25 0.30];
        totE=sum(moveE)+sum(pushE);
        title(axBar,sprintf('Energy  (Total=%.0f)',totE), ...
            'Color',[0.85 0.90 1.0],'FontSize',9);
        legend(axBar,{'Move','Push'},'TextColor',[0.78 0.82 0.90], ...
            'Color',[0.12 0.12 0.15],'EdgeColor',[0.28 0.30 0.35], ...
            'FontSize',7,'Location','northwest');
    end

    function liveUpdateMetrics(m,n)
        if ~isvalid(axInfo), return; end
        cla(axInfo);
        axInfo.XLim=[0 1]; axInfo.YLim=[0 1];
        lns={
            sprintf('Robots      %d',    n)
            sprintf('Success     %d/%d  (%.0f%%)',m.successCount,n,m.successRate*100)
            sprintf('Total E     %.1f',  m.totalEnergy)
            sprintf('Avg E       %.1f',  m.avgEnergy)
            sprintf('Planned     %d removals', m.removals)
            sprintf('Steps       %d',    m.steps)
        };
        clrs=[0.65 0.80 1.00;0.35 0.95 0.48;1.00 0.68 0.25;
              0.80 0.60 1.00;1.00 0.88 0.10;0.55 0.85 0.60];
        for li=1:numel(lns)
            text(axInfo,0.04,1-li*0.145,lns{li},'Color',clrs(li,:), ...
                'FontSize',10.5,'FontName','Courier New','FontWeight','bold', ...
                'Units','normalized','Interpreter','none');
        end
        axInfo.XAxis.Visible='off'; axInfo.YAxis.Visible='off';
    end

end % MAPFRO_3D_Visualizer

%% ══════════════════════════════════════════════════════════════════════
%% TOP-LEVEL HELPERS (no closure needed)
%% ══════════════════════════════════════════════════════════════════════

function nextPos = getNextPos(robot)
    nextPos=[];
    if robot.isFinished()||isempty(robot.plannedPath), return; end
    ns=robot.pathStep+1;
    if ns<=size(robot.plannedPath,1), nextPos=robot.plannedPath(ns,:); end
end

function txt = makeInfoText(env,nR,plan,m)
    txt=sprintf('  %dx%d  |  Robots:%d  |  Pits:%d  Sandbags:%d  |  Removed:%d  |  Success:%.0f%%  |  Energy:%.0f', ...
        env.rows,env.cols,nR, ...
        size(env.pitPositions,1),size(env.sandbagPositions,1), ...
        numel(plan),m.successRate*100,m.totalEnergy);
end

%% ══════════════════════════════════════════════════════════════════════
%% TURTLEBOT3-STYLE DIFFERENTIAL DRIVE ROBOT
%% Modelled after real ROS/Gazebo TurtleBot3 Burger:
%%   - Circular body platform (large cylinder)
%%   - Two large rubber drive wheels on sides
%%   - Small front + rear caster balls
%%   - Tall LIDAR mast with spinning RPLidar A1 drum
%%   - Coloured LED ring around base
%%   - Robot fills ~75% of grid cell — clearly visible
%% ══════════════════════════════════════════════════════════════════════

function [hBody, hLidar] = drawCar(ax, cx, cy, col, lidarDeg)
hBody = gobjects(40,1); hi=1;

%% ── Dimensions — LARGE and clearly visible ───────────────────────────
R_body = 0.62;   % platform radius  (grid cell = 1.0, robot fills ~90%)
H_body = 0.30;   % platform height  — thick and bold
Z_body = 0.18;   % chassis bottom Z
N      = 24;     % circle segments

th = linspace(0,2*pi,N+1); th=th(1:end-1);

%% ── 1. MAIN BODY CYLINDER ────────────────────────────────────────────
% Side wall
[xc,yc,zc] = cylinder(R_body, N);
zc(1,:) = Z_body;  zc(2,:) = Z_body + H_body;
hBody(hi) = surf(ax, cx+xc, cy+yc, zc, ...
    'FaceColor',col, 'EdgeColor','none', ...
    'FaceLighting','gouraud','AmbientStrength',0.38,...
    'DiffuseStrength',0.78,'SpecularStrength',0.30); hi=hi+1;

% Top cap
topZ = Z_body + H_body;
hBody(hi) = patch(ax, cx+R_body*cos(th), cy+R_body*sin(th), topZ*ones(1,N), ...
    'FaceColor',min(col+[0.18 0.18 0.18],1), 'EdgeColor','none', ...
    'FaceLighting','flat','AmbientStrength',0.55); hi=hi+1;

% Bottom cap (dark)
hBody(hi) = patch(ax, cx+R_body*cos(th), cy+R_body*sin(th), Z_body*ones(1,N), ...
    'FaceColor',col*0.45, 'EdgeColor','none', ...
    'FaceLighting','flat','AmbientStrength',0.30); hi=hi+1;

%% ── 2. LED ACCENT RING (glowing band around base of body) ────────────
ledZ1 = Z_body + H_body*0.10;
ledZ2 = Z_body + H_body*0.28;
[xl,yl,zl] = cylinder(R_body+0.012, N);
zl(1,:)=ledZ1; zl(2,:)=ledZ2;
hBody(hi) = surf(ax, cx+xl, cy+yl, zl, ...
    'FaceColor',min(col+[0.35 0.35 0.35],1), 'EdgeColor','none', ...
    'FaceLighting','flat','AmbientStrength',0.90); hi=hi+1;

%% ── 3. SECOND PLATFORM LAYER (upper deck) ────────────────────────────
R_deck = R_body*0.76; H_deck = H_body*0.50;
Z_deck = topZ + 0.02;
[xd,yd,zd] = cylinder(R_deck, N);
zd(1,:)=Z_deck; zd(2,:)=Z_deck+H_deck;
hBody(hi) = surf(ax, cx+xd, cy+yd, zd, ...
    'FaceColor',col*0.80, 'EdgeColor','none', ...
    'FaceLighting','gouraud','AmbientStrength',0.40,'DiffuseStrength',0.70); hi=hi+1;
% Deck top cap
deckTopZ = Z_deck+H_deck;
hBody(hi) = patch(ax, cx+R_deck*cos(th), cy+R_deck*sin(th), deckTopZ*ones(1,N), ...
    'FaceColor',col*0.70+0.12, 'EdgeColor','none', ...
    'FaceLighting','flat','AmbientStrength',0.50); hi=hi+1;

%% ── 4. TWO DRIVE WHEELS ──────────────────────────────────────────────
W_rad = 0.230;  % wheel radius — large and visible
W_wid = 0.140;  % wheel width
W_off = R_body + W_wid*0.40;  % lateral offset
Z_whl = W_rad;  % wheel centre height = radius (touches ground)
thW   = linspace(0,2*pi,21);
tyreCol = [0.06 0.06 0.06];
rimCol  = [0.55 0.58 0.62];

for side = [-1, 1]
    wy = cy + side*W_off;
    % Tyre cylinder (axis along world-Y)
    [xw,yw,zw] = cylinder(W_rad, 20);
    % cylinder default axis = Z; remap axis to Y:
    % new_X = z*W_wid - W_wid/2, new_Y = x, new_Z = y + W_rad
    hBody(hi) = surf(ax, ...
        cx + zw*W_wid - W_wid/2, ...
        wy + xw, ...
        Z_whl + yw, ...
        'FaceColor',tyreCol,'EdgeColor','none',...
        'FaceLighting','gouraud','AmbientStrength',0.25,'DiffuseStrength',0.48); hi=hi+1;
    % Tyre sidewalls
    for s2=[-1 1]
        hBody(hi)=patch(ax, cx+s2*W_wid/2*ones(1,21), wy+W_rad*cos(thW), Z_whl+W_rad*sin(thW),...
            'FaceColor',tyreCol,'EdgeColor','none','FaceLighting','flat','AmbientStrength',0.25); hi=hi+1;
    end
    % Rim (silver hub)
    rRim=W_rad*0.56;
    hBody(hi)=patch(ax, cx+(side*W_off+side*W_wid/2-side*0.008)*ones(1,21), ...
        wy+rRim*cos(thW), Z_whl+rRim*sin(thW),...
        'FaceColor',rimCol,'EdgeColor','none',...
        'FaceLighting','gouraud','AmbientStrength',0.60); hi=hi+1;
    % Hub bolt circle
    for b=0:4
        ba=b*2*pi/5; br=rRim*0.50;
        bxv=cx+(side*W_off+side*W_wid/2)*ones(1,8);
        byv=wy+br*cos(ba)+0.022*cos(linspace(0,2*pi,8));
        bzv=Z_whl+br*sin(ba)+0.022*sin(linspace(0,2*pi,8));
        hBody(hi)=patch(ax,bxv,byv,bzv,...
            'FaceColor',[0.88 0.90 0.92],'EdgeColor','none','FaceLighting','flat'); hi=hi+1;
    end
end

%% ── 5. CASTER BALLS (front + rear) ──────────────────────────────────
casterR = 0.055;
casterZ = casterR;
[xs,ys,zs] = sphere(8);
for cpos = [R_body*0.72, -R_body*0.72]
    hBody(hi) = surf(ax, cx+cpos+casterR*xs, cy+casterR*ys, casterZ+casterR*zs, ...
        'FaceColor',[0.50 0.52 0.55],'EdgeColor','none',...
        'FaceLighting','gouraud','AmbientStrength',0.45); hi=hi+1;
end

%% ── 6. LIDAR MAST ────────────────────────────────────────────────────
mastR = 0.065; mastH = 0.55;
mastZb = deckTopZ;
[xm,ym,zm] = cylinder(mastR, 12);
zm(1,:)=mastZb; zm(2,:)=mastZb+mastH;
hBody(hi) = surf(ax, cx+xm, cy+ym, zm, ...
    'FaceColor',[0.55 0.58 0.62],'EdgeColor','none',...
    'FaceLighting','gouraud','AmbientStrength',0.42,'DiffuseStrength',0.62); hi=hi+1;

%% ── 7. RPLIDAR DRUM (black cylinder + red window) ────────────────────
lidarZ  = mastZb + mastH;
drumR   = 0.120; drumH2 = 0.095;
[xld,yld,zld] = cylinder(drumR, 16);
zld(1,:)=lidarZ; zld(2,:)=lidarZ+drumH2;
hBody(hi) = surf(ax, cx+xld, cy+yld, zld, ...
    'FaceColor',[0.10 0.10 0.12],'EdgeColor','none',...
    'FaceLighting','gouraud','AmbientStrength',0.35,'DiffuseStrength',0.55); hi=hi+1;
% Top cap of drum
lidarTopZ = lidarZ+drumH2;
hBody(hi) = patch(ax, cx+drumR*cos(th(1:16)), cy+drumR*sin(th(1:16)), lidarTopZ*ones(1,16), ...
    'FaceColor',[0.15 0.15 0.18],'EdgeColor','none','FaceLighting','flat'); hi=hi+1;
% Red sensor window strip (front arc)
arcA = linspace(-pi/3, pi/3, 10);
xArc = [cx+drumR*cos(arcA), cx+(drumR+0.01)*cos(fliplr(arcA))];
yArc = [cy+drumR*sin(arcA), cy+(drumR+0.01)*sin(fliplr(arcA))];
zA1  = (lidarZ+drumH2*0.15)*ones(1,10);
zA2  = (lidarZ+drumH2*0.85)*ones(1,10);
% Side face of red arc
for ai=1:9
    hBody(hi)=patch(ax,...
        [xArc(ai) xArc(ai+1) xArc(ai+1) xArc(ai)],...
        [yArc(ai) yArc(ai+1) yArc(ai+1) yArc(ai)],...
        [zA1(ai)  zA1(ai+1)  zA2(ai+1)  zA2(ai)], ...
        'FaceColor',[0.95 0.08 0.08],'EdgeColor','none','FaceLighting','flat','AmbientStrength',0.85); hi=hi+1;
end

%% ── 8. SPINNING LIDAR BEAM (green) ───────────────────────────────────
scanZ   = lidarZ + drumH2*0.55;
ang     = deg2rad(lidarDeg);
scanR   = 2.20;  % visible scan radius
fanA    = linspace(ang, ang+deg2rad(115), 16);
hFan    = patch(ax, [cx, cx+scanR*cos(fanA)], [cy, cy+scanR*sin(fanA)], ...
    scanZ*ones(1,17), ...
    'FaceColor',[0.05 0.95 0.05],'EdgeColor','none',...
    'FaceAlpha',0.08,'FaceLighting','flat');
hBeam   = plot3(ax, [cx cx+scanR*cos(ang)], [cy cy+scanR*sin(ang)], [scanZ scanZ], ...
    '-','Color',[0.05 1.00 0.05 0.72],'LineWidth',2.0);
hLidar  = [hFan; hBeam];
end


%% ══════════════════════════════════════════════════════════════════════
%% WORLD DRAWING
%% ══════════════════════════════════════════════════════════════════════

function drawWarehouseFloor(ax,R,C)
% Dark cinematic warehouse concrete floor — like real Gazebo/RViz
for r=0:R-1
    for c=0:C-1
        if mod(r+c,2)==0, col=[0.30 0.31 0.33];
        else,              col=[0.26 0.27 0.29]; end
        patch(ax,'XData',[c c+1 c+1 c],'YData',[r r r+1 r+1],...
            'ZData',[0 0 0 0],'FaceColor',col,'EdgeColor','none',...
            'FaceLighting','flat','AmbientStrength',0.75);
    end
end
% Grout lines
gc=[0.18 0.19 0.21];
for r=0:R, plot3(ax,[0 C],[r r],[0.003 0.003],'-','Color',[gc,0.60],'LineWidth',0.5); end
for c=0:C, plot3(ax,[c c],[0 R],[0.003 0.003],'-','Color',[gc,0.60],'LineWidth',0.5); end
% Bright yellow safety hazard border
yw=[1.00 0.85 0.00]; bz=0.005; bw=0.18;
for c=0:C-1
    patch(ax,[c c+1 c+1 c],[0 0 bw bw],[bz bz bz bz],...
        'FaceColor',yw,'EdgeColor','none','FaceAlpha',0.90,'FaceLighting','flat');
    patch(ax,[c c+1 c+1 c],[R-bw R-bw R R],[bz bz bz bz],...
        'FaceColor',yw,'EdgeColor','none','FaceAlpha',0.90,'FaceLighting','flat');
end
for r=0:R-1
    patch(ax,[0 bw bw 0],[r r r+1 r+1],[bz bz bz bz],...
        'FaceColor',yw,'EdgeColor','none','FaceAlpha',0.90,'FaceLighting','flat');
    patch(ax,[C-bw C C C-bw],[r r r+1 r+1],[bz bz bz bz],...
        'FaceColor',yw,'EdgeColor','none','FaceAlpha',0.90,'FaceLighting','flat');
end
end

function drawPerimeterWalls(ax,R,C)
% Dark industrial concrete walls with bright trim
wH=2.5; wT=0.45;
col=[0.22 0.24 0.28]; trim=[0.40 0.42 0.48];
drawBevelBox(ax,C/2,   R+wT/2, wH/2, C+wT*2,wT,  wH, col,trim);
drawBevelBox(ax,C/2,   -wT/2,  wH/2, C+wT*2,wT,  wH, col,trim);
drawBevelBox(ax,C+wT/2, R/2,   wH/2, wT,    R,   wH, col,trim);
drawBevelBox(ax,-wT/2,  R/2,   wH/2, wT,    R,   wH, col,trim);
end

function drawStaticObstacles(ax,env,R)
% Dark concrete block obstacles — varying heights for realism
[sr,sc]=find(env.grid==env.STATIC);
for i=1:numel(sr)
    wx=sc(i)-0.5; wy=R-sr(i)+0.5;
    h=1.0+0.5*mod(sr(i)*3+sc(i)*7,4)*0.22;
    % Vary colour slightly per block
    shade=0.20+0.06*mod(sr(i)+sc(i),3);
    col=[shade shade+0.02 shade+0.04];
    trim=col+0.18;
    drawBevelBox(ax,wx,wy,h/2,0.90,0.90,h,col,trim);
end
end

function drawPits(ax,env,R)
[pr,pc]=find(env.grid==env.PIT);
for i=1:numel(pr)
    wx=pc(i)-0.5; wy=R-pr(i)+0.5;
    patch(ax,wx+0.44*[-1 1 1 -1],wy+0.44*[-1 -1 1 1],[-0.08 -0.08 -0.08 -0.08], ...
        'FaceColor',[0.06 0.24 0.70],'EdgeColor','none','FaceLighting','flat');
    patch(ax,wx+0.28*[-1 1 1 -1],wy+0.28*[-1 -1 1 1],[-0.04 -0.04 -0.04 -0.04], ...
        'FaceColor',[0.15 0.45 0.90],'EdgeColor','none','FaceAlpha',0.80,'FaceLighting','flat');
    th=linspace(0,2*pi,17); rg=0.47;
    patch(ax,wx+rg*cos(th),wy+rg*sin(th),zeros(1,17)+0.005, ...
        'FaceColor','none','EdgeColor',[0.25 0.60 1.00],'EdgeAlpha',0.75,'LineWidth',1.8);
    text(ax,wx,wy,0.04,'P','Color',[0.50 0.80 1.00],'FontSize',7,'FontWeight','bold', ...
        'HorizontalAlignment','center','VerticalAlignment','middle');
end
end

function drawSandbags(ax,env,R)
[sbr,sbc]=find(env.grid==env.SANDBAG);
for i=1:numel(sbr)
    wx=sbc(i)-0.5; wy=R-sbr(i)+0.5;
    drawOneBag(ax,wx,    wy,     0.08,0.80,0.68,0.18,[0.50 0.35 0.10]);
    drawOneBag(ax,wx+0.03,wy-0.02,0.26,0.74,0.63,0.17,[0.58 0.40 0.14]);
    drawOneBag(ax,wx-0.02,wy+0.03,0.43,0.64,0.53,0.15,[0.66 0.46 0.18]);
    for zb=[0.10,0.28,0.44]
        plot3(ax,wx+0.36*[-1 1],wy*[1 1],[zb zb],'-','Color',[0.20 0.14 0.04],'LineWidth',1.0);
    end
end
end

function drawOneBag(ax,cx,cy,cz,dx,dy,dz,col)
x=cx+dx/2*[-1 1 1 -1 -1 1 1 -1];
y=cy+dy/2*[-1 -1 1 1 -1 -1 1 1];
z=cz+dz/2*[-1 -1 -1 -1 1 1 1 1];
f=[1 2 3 4;5 6 7 8;1 2 6 5;3 4 8 7;1 4 8 5;2 3 7 6];
patch(ax,'Vertices',[x;y;z]','Faces',f,'FaceColor',col,'EdgeColor',col*0.45, ...
    'EdgeAlpha',0.30,'FaceLighting','gouraud','AmbientStrength',0.36, ...
    'DiffuseStrength',0.63,'SpecularStrength',0.06);
end

function gh = drawGoalMarkersTagged(ax,robots,R,PAL,nR)
gh = gobjects(nR*3,1); gi=1;
for i=1:nR
    g=robots{i}.goalPos; wx=g(2)-0.5; wy=R-g(1)+0.5;
    col=PAL(mod(i-1,size(PAL,1))+1,:);
    th=linspace(0,2*pi,5); rp=0.36;
    gh(gi)=patch(ax,wx+rp*cos(th+pi/4),wy+rp*sin(th+pi/4),zeros(1,5)+0.004, ...
        'FaceColor',col*0.40+0.20,'EdgeColor',col,'EdgeAlpha',0.88, ...
        'LineWidth',1.8,'FaceAlpha',0.50,'FaceLighting','flat'); gi=gi+1;
    gh(gi)=plot3(ax,[wx wx],[wy wy],[0.0 0.50],'-','Color',col,'LineWidth',2.2); gi=gi+1;
    gh(gi)=text(ax,wx,wy,0.82,sprintf('G%d',i),'Color','w','FontSize',7.5, ...
        'FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','bottom'); gi=gi+1;
end
end

function ph = drawPathLinesTagged(ax,paths,R,PAL,nR)
ph = gobjects(nR,1);
for i=1:nR
    pth=paths{i};
    if isempty(pth)||size(pth,1)<2
        ph(i)=plot3(ax,NaN,NaN,NaN);
        continue;
    end
    col=PAL(mod(i-1,size(PAL,1))+1,:);
    wx=pth(:,2)-0.5; wy=R-pth(:,1)+0.5;
    ph(i)=plot3(ax,wx,wy,ones(size(wx))*0.012,'--','Color',[col,0.25],'LineWidth',0.9);
end
end

function drawGoalMarkers(ax,robots,R,PAL,nR)
for i=1:nR
    g=robots{i}.goalPos; wx=g(2)-0.5; wy=R-g(1)+0.5;
    col=PAL(mod(i-1,size(PAL,1))+1,:);
    th=linspace(0,2*pi,5); rp=0.36;
    patch(ax,wx+rp*cos(th+pi/4),wy+rp*sin(th+pi/4),zeros(1,5)+0.004, ...
        'FaceColor',col*0.40+0.20,'EdgeColor',col,'EdgeAlpha',0.88, ...
        'LineWidth',1.8,'FaceAlpha',0.50,'FaceLighting','flat');
    plot3(ax,[wx wx],[wy wy],[0.0 0.50],'-','Color',col,'LineWidth',2.2);
    drawSphere(ax,wx,wy,0.58,0.085,col);
    text(ax,wx,wy,0.82,sprintf('G%d',i),'Color','w','FontSize',7.5, ...
        'FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','bottom');
end
end

function drawSphere(ax,cx,cy,cz,r,col)
[xs,ys,zs]=sphere(10);
surf(ax,cx+r*xs,cy+r*ys,cz+r*zs,'FaceColor',col,'EdgeColor','none', ...
    'FaceLighting','gouraud','AmbientStrength',0.45,'DiffuseStrength',0.70,'SpecularStrength',0.45);
end

function drawPathLines(ax,paths,R,PAL,nR)
for i=1:nR
    pth=paths{i};
    if isempty(pth)||size(pth,1)<2, continue; end
    col=PAL(mod(i-1,size(PAL,1))+1,:);
    wx=pth(:,2)-0.5; wy=R-pth(:,1)+0.5;
    plot3(ax,wx,wy,ones(size(wx))*0.012,'--','Color',[col,0.25],'LineWidth',0.9);
end
end

function drawAssignmentArrows(ax,plan,R)
for ri=1:numel(plan)
    rp=plan(ri);
    if ~strcmp(rp.type,'pit')||isequal(rp.sandbagPos,[0 0]), continue; end
    sx=rp.sandbagPos(2)-0.5; sy=R-rp.sandbagPos(1)+0.5;
    px=rp.pos(2)-0.5; py=R-rp.pos(1)+0.5;
    plot3(ax,[sx px],[sy py],[0.80 0.80],'->','Color',[1.0 0.88 0.10 0.80], ...
        'LineWidth',2.0,'MarkerSize',6);
end
end

function drawBevelBox(ax,cx,cy,cz,dx,dy,dz,col,trim)
x=cx+dx/2*[-1 1 1 -1 -1 1 1 -1];
y=cy+dy/2*[-1 -1 1 1 -1 -1 1 1];
z=cz+dz/2*[-1 -1 -1 -1 1 1 1 1];
f=[1 2 3 4;5 6 7 8;1 2 6 5;3 4 8 7;1 4 8 5;2 3 7 6];
patch(ax,'Vertices',[x;y;z]','Faces',f,'FaceColor',col,'EdgeColor','none', ...
    'FaceLighting','gouraud','AmbientStrength',0.32,'DiffuseStrength',0.67,'SpecularStrength',0.12);
topV=[cx-dx/2 cy-dy/2 cz+dz/2;
      cx+dx/2 cy-dy/2 cz+dz/2;
      cx+dx/2 cy+dy/2 cz+dz/2;
      cx-dx/2 cy+dy/2 cz+dz/2;
      cx-dx/2 cy-dy/2 cz+dz/2];
plot3(ax,topV(:,1),topV(:,2),topV(:,3),'-','Color',[trim,0.58],'LineWidth',0.8);
end

function pal=robotPalette()
pal=[0.15 0.50 1.00;1.00 0.35 0.05;0.10 0.78 0.38;0.88 0.15 0.25;
     0.72 0.15 0.85;0.90 0.80 0.05;0.05 0.80 0.80;1.00 0.48 0.65;
     0.50 0.85 0.20;0.35 0.25 0.88;0.88 0.52 0.08;0.08 0.62 0.88;
     0.78 0.78 0.18;0.18 0.78 0.75;0.78 0.18 0.75;0.55 0.88 0.45;
     0.88 0.55 0.45;0.45 0.55 0.88;0.68 0.38 0.18;0.38 0.68 0.18];
end