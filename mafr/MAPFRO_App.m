classdef MAPFRO_App < matlab.apps.AppBase

    % MAPFRO_App - Professional App Designer GUI for MAPF-RO Simulation
    %
    % Layout (Nav2/RViz inspired dark theme):
    %   LEFT  (60%): Live grid with animated robots, paths, obstacles
    %   RIGHT TOP   : Energy bar chart (move vs push per robot)
    %   RIGHT MID   : Energy over time lines
    %   RIGHT BOTTOM: Status panel + controls
    %   BOTTOM BAR  : Play/Pause/Step/Reset | Speed slider | Config
    %
    % Usage: MAPFRO_App

    %% ── App Designer generated properties ───────────────────────────────
    properties (Access = public)
        UIFigure            matlab.ui.Figure

        % ── Panels ───────────────────────────────────────────────────────
        GridPanel           matlab.ui.container.Panel
        RightPanel          matlab.ui.container.Panel
        ControlPanel        matlab.ui.container.Panel

        % ── Grid axes ────────────────────────────────────────────────────
        GridAxes            matlab.ui.control.UIAxes

        % ── Right panel axes ─────────────────────────────────────────────
        EnergyAxes          matlab.ui.control.UIAxes
        ProgressAxes        matlab.ui.control.UIAxes
        StatusAxes          matlab.ui.control.UIAxes

        % ── Control buttons ───────────────────────────────────────────────
        BtnPlay             matlab.ui.control.Button
        BtnPause            matlab.ui.control.Button
        BtnStep             matlab.ui.control.Button
        BtnReset            matlab.ui.control.Button
        BtnBatch            matlab.ui.control.Button

        % ── Config controls ───────────────────────────────────────────────
        DropGridSize        matlab.ui.control.DropDown
        SpinnerRobots       matlab.ui.control.Spinner
        SpinnerK            matlab.ui.control.Spinner
        SpinnerSeed         matlab.ui.control.Spinner
        SliderSpeed         matlab.ui.control.Slider
        ToggleRO            matlab.ui.control.StateButton
        ToggleTraffic       matlab.ui.control.StateButton

        % ── Labels ────────────────────────────────────────────────────────
        LblSpeed            matlab.ui.control.Label
        LblStatus           matlab.ui.control.Label
        LblTitle            matlab.ui.control.Label
    end

    %% ── App state ───────────────────────────────────────────────────────
    properties (Access = private)
        sim         Simulation
        isRunning   logical = false
        isPaused    logical = false
        animTimer   timer
    end

    %% ════════════════════════════════════════════════════════════════════
    %  STARTUP
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = private)

        function startupFcn(app)
            app.buildUI();
            app.applyDarkTheme();
            app.setStatus('Ready — press ▶ Play to start', [0.6 0.8 1.0]);
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  UI CONSTRUCTION
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = private)

        function buildUI(app)
            % ── Main figure ──────────────────────────────────────────────
            app.UIFigure = uifigure('Name', 'MAPF-RO Simulator', ...
                'Position', [30 30 1300 760], ...
                'Color', [0.10 0.10 0.12], ...
                'Resize', 'on');
            app.UIFigure.CloseRequestFcn = @(~,~) app.onClose();

            % ── Title bar ────────────────────────────────────────────────
            app.LblTitle = uilabel(app.UIFigure, ...
                'Text', '  MAPF-RO  |  Multi-Robot Path Finding with Removable Obstacles', ...
                'Position', [0 725 1300 35], ...
                'FontSize', 13, 'FontWeight', 'bold', ...
                'FontColor', [0.85 0.90 1.00], ...
                'BackgroundColor', [0.08 0.10 0.18], ...
                'HorizontalAlignment', 'left');

            % ── Grid panel (left 62%) ────────────────────────────────────
            app.GridPanel = uipanel(app.UIFigure, ...
                'Position', [5 60 795 660], ...
                'BackgroundColor', [0.10 0.10 0.12], ...
                'BorderType', 'none');

            app.GridAxes = uiaxes(app.GridPanel, ...
                'Position', [5 5 785 650], ...
                'Color', [0.13 0.13 0.15], ...
                'XColor', [0.4 0.4 0.4], ...
                'YColor', [0.4 0.4 0.4], ...
                'GridColor', [0.25 0.25 0.25], ...
                'XGrid', 'on', 'YGrid', 'on');
            title(app.GridAxes, 'MAPF-RO Environment', ...
                'Color', [0.85 0.85 0.90], 'FontSize', 11, 'FontWeight', 'bold');

            % ── Right panel ───────────────────────────────────────────────
            app.RightPanel = uipanel(app.UIFigure, ...
                'Position', [805 60 490 660], ...
                'BackgroundColor', [0.10 0.10 0.12], ...
                'BorderType', 'none');

            % Energy bars
            app.EnergyAxes = uiaxes(app.RightPanel, ...
                'Position', [5 415 480 235], ...
                'Color', [0.13 0.13 0.15], ...
                'XColor', [0.65 0.65 0.65], ...
                'YColor', [0.65 0.65 0.65], ...
                'GridColor', [0.28 0.28 0.28], ...
                'YGrid', 'on');
            title(app.EnergyAxes, 'Energy per Robot', ...
                'Color', [0.85 0.85 0.90], 'FontSize', 10);
            ylabel(app.EnergyAxes, 'Energy', ...
                'Color', [0.65 0.65 0.65], 'FontSize', 8);

            % Energy progress
            app.ProgressAxes = uiaxes(app.RightPanel, ...
                'Position', [5 215 480 190], ...
                'Color', [0.13 0.13 0.15], ...
                'XColor', [0.65 0.65 0.65], ...
                'YColor', [0.65 0.65 0.65], ...
                'GridColor', [0.28 0.28 0.28], ...
                'XGrid', 'on', 'YGrid', 'on');
            title(app.ProgressAxes, 'Energy Over Time', ...
                'Color', [0.85 0.85 0.90], 'FontSize', 10);
            xlabel(app.ProgressAxes, 'Step', ...
                'Color', [0.65 0.65 0.65], 'FontSize', 8);

            % Status text
            app.StatusAxes = uiaxes(app.RightPanel, ...
                'Position', [5 5 480 200], ...
                'Color', [0.10 0.10 0.12], ...
                'Visible', 'on', ...
                'XColor', [0.10 0.10 0.12], ...
                'YColor', [0.10 0.10 0.12]);
            app.StatusAxes.XAxis.Visible = 'off';
            app.StatusAxes.YAxis.Visible = 'off';
            app.StatusAxes.XLim = [0 1];
            app.StatusAxes.YLim = [0 1];

            % ── Control panel (bottom bar) ────────────────────────────────
            app.ControlPanel = uipanel(app.UIFigure, ...
                'Position', [0 0 1300 58], ...
                'BackgroundColor', [0.08 0.09 0.14], ...
                'BorderType', 'none');

            % Play button
            app.BtnPlay = uibutton(app.ControlPanel, 'push', ...
                'Text', '▶  Play', ...
                'Position', [8 10 90 36], ...
                'BackgroundColor', [0.15 0.55 0.25], ...
                'FontColor', [1 1 1], 'FontSize', 12, 'FontWeight', 'bold', ...
                'ButtonPushedFcn', @(~,~) app.onPlay());

            % Pause button
            app.BtnPause = uibutton(app.ControlPanel, 'push', ...
                'Text', '⏸  Pause', ...
                'Position', [104 10 90 36], ...
                'BackgroundColor', [0.55 0.45 0.10], ...
                'FontColor', [1 1 1], 'FontSize', 12, 'FontWeight', 'bold', ...
                'ButtonPushedFcn', @(~,~) app.onPause());

            % Step button
            app.BtnStep = uibutton(app.ControlPanel, 'push', ...
                'Text', '⏭  Step', ...
                'Position', [200 10 85 36], ...
                'BackgroundColor', [0.15 0.35 0.60], ...
                'FontColor', [1 1 1], 'FontSize', 12, 'FontWeight', 'bold', ...
                'ButtonPushedFcn', @(~,~) app.onStep());

            % Reset button
            app.BtnReset = uibutton(app.ControlPanel, 'push', ...
                'Text', '↺  Reset', ...
                'Position', [291 10 85 36], ...
                'BackgroundColor', [0.50 0.15 0.15], ...
                'FontColor', [1 1 1], 'FontSize', 12, 'FontWeight', 'bold', ...
                'ButtonPushedFcn', @(~,~) app.onReset());

            % Divider
            uilabel(app.ControlPanel, 'Text', '│', ...
                'Position', [382 8 15 38], ...
                'FontColor', [0.35 0.35 0.45], 'FontSize', 20);

            % Grid size
            uilabel(app.ControlPanel, 'Text', 'Grid:', ...
                'Position', [398 28 35 18], ...
                'FontColor', [0.70 0.75 0.85], 'FontSize', 9);
            app.DropGridSize = uidropdown(app.ControlPanel, ...
                'Items', {'small (20×20)', 'medium (30×30)', 'large (40×40)'}, ...
                'Value', 'small (20×20)', ...
                'Position', [398 8 130 22], ...
                'BackgroundColor', [0.18 0.18 0.22], ...
                'FontColor', [0.85 0.88 0.95], 'FontSize', 9);

            % Robots spinner
            uilabel(app.ControlPanel, 'Text', 'Robots:', ...
                'Position', [535 28 48 18], ...
                'FontColor', [0.70 0.75 0.85], 'FontSize', 9);
            app.SpinnerRobots = uispinner(app.ControlPanel, ...
                'Value', 5, 'Limits', [1 25], 'Step', 1, ...
                'Position', [535 8 60 22], ...
                'BackgroundColor', [0.18 0.18 0.22], ...
                'FontColor', [0.85 0.88 0.95], 'FontSize', 9);

            % k spinner
            uilabel(app.ControlPanel, 'Text', 'Push k:', ...
                'Position', [602 28 45 18], ...
                'FontColor', [0.70 0.75 0.85], 'FontSize', 9);
            app.SpinnerK = uispinner(app.ControlPanel, ...
                'Value', 2, 'Limits', [1 10], 'Step', 1, ...
                'Position', [602 8 55 22], ...
                'BackgroundColor', [0.18 0.18 0.22], ...
                'FontColor', [0.85 0.88 0.95], 'FontSize', 9);

            % Seed spinner
            uilabel(app.ControlPanel, 'Text', 'Seed:', ...
                'Position', [664 28 38 18], ...
                'FontColor', [0.70 0.75 0.85], 'FontSize', 9);
            app.SpinnerSeed = uispinner(app.ControlPanel, ...
                'Value', 42, 'Limits', [0 9999], 'Step', 1, ...
                'Position', [664 8 60 22], ...
                'BackgroundColor', [0.18 0.18 0.22], ...
                'FontColor', [0.85 0.88 0.95], 'FontSize', 9);

            % Divider
            uilabel(app.ControlPanel, 'Text', '│', ...
                'Position', [730 8 15 38], ...
                'FontColor', [0.35 0.35 0.45], 'FontSize', 20);

            % Speed slider
            uilabel(app.ControlPanel, 'Text', 'Speed:', ...
                'Position', [748 32 48 16], ...
                'FontColor', [0.70 0.75 0.85], 'FontSize', 9);
            app.SliderSpeed = uislider(app.ControlPanel, ...
                'Limits', [0.01 0.5], 'Value', 0.15, ...
                'Position', [748 18 130 3], ...
                'MajorTicks', [], 'MinorTicks', []);
            app.LblSpeed = uilabel(app.ControlPanel, ...
                'Text', '0.15s', ...
                'Position', [884 28 40 16], ...
                'FontColor', [0.65 0.70 0.80], 'FontSize', 8);
            app.SliderSpeed.ValueChangedFcn = @(s,~) app.onSpeedChange(s);

            % Divider
            uilabel(app.ControlPanel, 'Text', '│', ...
                'Position', [928 8 15 38], ...
                'FontColor', [0.35 0.35 0.45], 'FontSize', 20);

            % RO toggle
            app.ToggleRO = uibutton(app.ControlPanel, 'state', ...
                'Text', 'MAPF-RO ON', ...
                'Value', true, ...
                'Position', [945 10 105 36], ...
                'BackgroundColor', [0.15 0.45 0.20], ...
                'FontColor', [0.85 1.00 0.85], ...
                'FontSize', 9, 'FontWeight', 'bold', ...
                'ValueChangedFcn', @(b,~) app.onROToggle(b));

            % Traffic toggle
            app.ToggleTraffic = uibutton(app.ControlPanel, 'state', ...
                'Text', 'Traffic OFF', ...
                'Value', false, ...
                'Position', [1056 10 105 36], ...
                'BackgroundColor', [0.20 0.20 0.30], ...
                'FontColor', [0.75 0.80 0.95], ...
                'FontSize', 9, 'FontWeight', 'bold', ...
                'ValueChangedFcn', @(b,~) app.onTrafficToggle(b));

            % 3D View button  
            uibutton(app.ControlPanel, 'push', ...
                'Text', '🌐 3D', ...
                'Position', [1167 10 55 36], ...
                'BackgroundColor', [0.12 0.28 0.48], ...
                'FontColor', [0.70 0.90 1.00], ...
                'FontSize', 10, 'FontWeight', 'bold', ...
                'ButtonPushedFcn', @(~,~) app.on3DView());

            % Batch button
            app.BtnBatch = uibutton(app.ControlPanel, 'push', ...
                'Text', '📊 Batch', ...
                'Position', [1228 10 60 36], ...
                'BackgroundColor', [0.25 0.20 0.40], ...
                'FontColor', [0.85 0.80 1.00], ...
                'FontSize', 10, 'FontWeight', 'bold', ...
                'ButtonPushedFcn', @(~,~) app.onBatch());

            % Status label
            app.LblStatus = uilabel(app.UIFigure, ...
                'Text', 'Ready', ...
                'Position', [5 62 790 18], ...
                'FontSize', 8.5, ...
                'FontColor', [0.55 0.75 0.55], ...
                'BackgroundColor', [0.08 0.09 0.12]);
        end

        function applyDarkTheme(app)
            % Extra polish on axes
            for ax = [app.GridAxes, app.EnergyAxes, app.ProgressAxes]
                ax.Color         = [0.13 0.13 0.15];
                ax.GridAlpha     = 0.25;
                ax.MinorGridAlpha= 0.12;
                ax.Box           = 'off';
            end
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  BUTTON CALLBACKS
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = private)

        function onPlay(app)
            if app.isRunning && ~app.isPaused
                return;   % already running
            end

            if ~app.isRunning
                % Fresh start: build simulation
                app.buildSimulation();
                if isempty(app.sim), return; end
                app.sim.plan();
                app.sim.showAnim = false;  % we drive animation ourselves
                app.isRunning = true;
                app.isPaused  = false;
                app.setStatus('Running...', [0.40 0.90 0.50]);
                app.renderFrame();
                app.startTimer();
            else
                % Resume from pause
                app.isPaused = false;
                app.setStatus('Running...', [0.40 0.90 0.50]);
                app.startTimer();
            end
        end

        function onPause(app)
            if ~app.isRunning, return; end
            app.isPaused = true;
            app.stopTimer();
            app.setStatus('Paused', [0.90 0.80 0.30]);
        end

        function onStep(app)
            % Single step (works whether paused or not started yet)
            if ~app.isRunning
                app.buildSimulation();
                if isempty(app.sim), return; end
                app.sim.plan();
                app.sim.showAnim = false;
                app.isRunning = true;
                app.isPaused  = true;
            end
            app.stopTimer();
            app.doOneStep();
            app.renderFrame();
        end

        function onReset(app)
            app.stopTimer();
            app.isRunning = false;
            app.isPaused  = false;
            app.sim       = [];
            app.clearAxes();
            app.setStatus('Reset — press ▶ Play to start', [0.6 0.8 1.0]);
        end

        function onSpeedChange(app, slider)
            v = slider.Value;
            app.LblSpeed.Text = sprintf('%.2fs', v);
            if ~isempty(app.animTimer) && isvalid(app.animTimer) && ...
               strcmp(app.animTimer.Running, 'on')
                app.stopTimer();
                app.startTimer();
            end
        end

        function onROToggle(app, btn)
            if btn.Value
                btn.Text            = 'MAPF-RO ON';
                btn.BackgroundColor = [0.15 0.45 0.20];
                btn.FontColor       = [0.85 1.00 0.85];
            else
                btn.Text            = 'Baseline';
                btn.BackgroundColor = [0.35 0.20 0.20];
                btn.FontColor       = [1.00 0.80 0.80];
            end
        end

        function onTrafficToggle(app, btn)
            if btn.Value
                btn.Text            = 'Traffic ON';
                btn.BackgroundColor = [0.35 0.25 0.10];
                btn.FontColor       = [1.00 0.85 0.45];
            else
                btn.Text            = 'Traffic OFF';
                btn.BackgroundColor = [0.20 0.20 0.30];
                btn.FontColor       = [0.75 0.80 0.95];
            end
            if app.isRunning
                app.renderFrame();
            end
        end

        function onBatch(app)
            app.stopTimer();
            app.setStatus('Running batch experiment... (this may take 1-3 min)', ...
                [0.90 0.75 0.30]);
            drawnow;
            gs = app.getGridSizeStr();
            tmpSim = Simulation();
            tmpSim.runBatch('robotCounts', [1 2 3 4 5 6 7 8 9 10], ...
                'kValues', [1 2 3 4], 'nEnvs', 5, 'gridSize', gs);
            app.setStatus('Batch complete! See new figure for results.', ...
                [0.40 0.90 0.50]);
        end

        function on3DView(app)
            % Launch 3D Gazebo-style visualizer
            if isempty(app.sim)
                % Auto-build if not yet started
                app.buildSimulation();
                if isempty(app.sim), return; end
                app.sim.plan();
            end
            app.setStatus('Opening 3D Gazebo-style world...', [0.50 0.80 1.00]);
            drawnow;
            try
                MAPFRO_3D_Visualizer(app.sim);
                app.setStatus('3D world open — press Animate in the 3D window', ...
                    [0.50 0.90 0.60]);
            catch e
                app.setStatus(['3D error: ' e.message], [1 0.4 0.4]);
            end
        end

        function onClose(app)
            app.stopTimer();
            delete(app.UIFigure);
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  SIMULATION DRIVING
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = private)

        function buildSimulation(app)
            gs    = app.getGridSizeStr();
            nR    = app.SpinnerRobots.Value;
            k     = app.SpinnerK.Value;
            seed  = app.SpinnerSeed.Value;
            useRO = app.ToggleRO.Value;

            app.setStatus(sprintf('Building %s environment with %d robots...', ...
                upper(gs), nR), [0.65 0.75 0.90]);
            drawnow;

            try
                app.sim = Simulation('size', gs, 'nRobots', nR, 'k', k, ...
                    'seed', seed, 'animate', false, 'useRO', useRO);
                app.sim.initialise();
                app.setStatus('Environment ready. Planning paths...', [0.65 0.75 0.90]);
                drawnow;
            catch e
                app.setStatus(['Error: ' e.message], [1.0 0.4 0.4]);
                app.sim = [];
            end
        end

        function startTimer(app)
            speed = app.SliderSpeed.Value;
            app.animTimer = timer(...
                'ExecutionMode', 'fixedRate', ...
                'Period',        max(speed, 0.05), ...
                'TimerFcn',      @(~,~) app.timerCallback());
            start(app.animTimer);
        end

        function stopTimer(app)
            if ~isempty(app.animTimer) && isvalid(app.animTimer)
                stop(app.animTimer);
                delete(app.animTimer);
            end
            app.animTimer = [];
        end

        function timerCallback(app)
            if ~app.isPaused && app.isRunning
                app.doOneStep();
                app.renderFrame();
            end
        end

        function doOneStep(app)
            if isempty(app.sim), return; end

            robots = app.sim.robots;
            allDone = all(cellfun(@(r) r.isFinished(), robots));

            if allDone
                app.stopTimer();
                app.isRunning = false;
                app.sim.collectMetrics();
                m = app.sim.metrics;
                app.setStatus(sprintf( ...
                    '✓ Done! %d/%d robots | Energy=%.0f | %d removals | %d steps', ...
                    m.successCount, m.nRobots, m.totalEnergy, ...
                    m.removals, m.steps), [0.30 0.95 0.45]);
                app.sim.printSummary();
                return;
            end

            % Collision-aware stepping
            occupied = app.sim.getCurrentPositions();
            for i = 1:numel(robots)
                r = robots{i};
                if r.isFinished(), continue; end

                nextPos = app.sim.getNextPos(r);
                if ~isempty(nextPos)
                    others = occupied;
                    others(i,:) = [];
                    if any(others(:,1)==nextPos(1) & others(:,2)==nextPos(2))
                        r.pause();
                    else
                        r.resume();
                    end
                end
                app.sim.executeRobotStep(r, i);
            end
            app.sim.stepCount = app.sim.stepCount + 1;
        end

        function renderFrame(app)
            if isempty(app.sim) || ~isvalid(app.UIFigure), return; end

            showTraffic = app.ToggleTraffic.Value;

            % ── Grid ─────────────────────────────────────────────────────
            cla(app.GridAxes);
            app.sim.env.visualize('ax', app.GridAxes, ...
                'traffic', showTraffic, ...
                'title', sprintf('MAPF-RO  |  Step %d  |  %s  |  %s', ...
                    app.sim.stepCount, ...
                    upper(app.getGridSizeStr()), ...
                    string(app.ToggleRO.Value).replace('true','RO').replace('false','Baseline')));

            hold(app.GridAxes, 'on');
            colors = app.getRobotColors();

            for i = 1:numel(app.sim.robots)
                r   = app.sim.robots{i};
                col = colors(i,:);

                % Planned path (very faint dashed)
                if ~isempty(r.plannedPath) && size(r.plannedPath,1)>1
                    plot(app.GridAxes, r.plannedPath(:,2), r.plannedPath(:,1), ...
                        '--','Color',[col,0.18],'LineWidth',0.9);
                end

                % Travelled trail
                if size(r.travelledPath,1) > 1
                    plot(app.GridAxes, r.travelledPath(:,2), r.travelledPath(:,1), ...
                        '-','Color',[col,0.65],'LineWidth',2.2);
                end

                % Goal
                scatter(app.GridAxes, r.goalPos(2), r.goalPos(1), 150, 'd', ...
                    'MarkerFaceColor', col*0.55+0.3, 'MarkerEdgeColor','w','LineWidth',1.5);
                text(app.GridAxes, r.goalPos(2), r.goalPos(1), sprintf('G%d',i), ...
                    'Color','w','FontSize',7,'FontWeight','bold',...
                    'HorizontalAlignment','center','VerticalAlignment','middle');

                % Robot body
                if r.isDone()
                    mk = 'p'; sz = 220;
                    scatter(app.GridAxes, r.currentPos(2), r.currentPos(1), sz, mk, ...
                        'MarkerFaceColor', col*0.4+0.4, 'MarkerEdgeColor','w','LineWidth',1.5);
                elseif r.isStuck()
                    scatter(app.GridAxes, r.currentPos(2), r.currentPos(1), 220, 'x', ...
                        'MarkerEdgeColor',[1 0.2 0.2],'LineWidth',3);
                elseif r.isWaiting()
                    scatter(app.GridAxes, r.currentPos(2), r.currentPos(1), 220, 's', ...
                        'MarkerFaceColor',col,'MarkerEdgeColor',[1 1 0.2],'LineWidth',2.5);
                else
                    scatter(app.GridAxes, r.currentPos(2), r.currentPos(1), 220, 'o', ...
                        'MarkerFaceColor',col,'MarkerEdgeColor','w','LineWidth',2.2);
                end

                text(app.GridAxes, r.currentPos(2), r.currentPos(1), sprintf('R%d',i), ...
                    'Color','w','FontSize',7,'FontWeight','bold',...
                    'HorizontalAlignment','center','VerticalAlignment','middle');
            end

            % Removal plan arrows
            for ri = 1:numel(app.sim.removalPlan)
                rp = app.sim.removalPlan(ri);
                if strcmp(rp.type,'pit') && ~isequal(rp.sandbagPos,[0 0])
                    plot(app.GridAxes, ...
                        [rp.sandbagPos(2), rp.pos(2)], ...
                        [rp.sandbagPos(1), rp.pos(1)], ...
                        '->','Color',[1 1 0.3 0.75],'LineWidth',1.8,'MarkerSize',5);
                end
            end
            hold(app.GridAxes,'off');

            % ── Energy bars ───────────────────────────────────────────────
            cla(app.EnergyAxes);
            n = numel(app.sim.robots);
            moveE = cellfun(@(r) r.energyMove, app.sim.robots);
            pushE = cellfun(@(r) r.energyPush, app.sim.robots);
            totalE = moveE + pushE;

            bData = [moveE; pushE]';
            b = bar(app.EnergyAxes, bData, 'stacked', 'EdgeColor','none');
            b(1).FaceColor = [0.25 0.60 1.00];
            if numel(b)>1, b(2).FaceColor = [1.00 0.45 0.10]; end

            app.EnergyAxes.XTickLabel = arrayfun(@(i) sprintf('R%d',i),1:n,'UniformOutput',false);
            app.EnergyAxes.Color    = [0.13 0.13 0.15];
            app.EnergyAxes.XColor   = [0.65 0.65 0.65];
            app.EnergyAxes.YColor   = [0.65 0.65 0.65];
            app.EnergyAxes.YGrid    = 'on';
            app.EnergyAxes.GridColor= [0.28 0.28 0.28];
            title(app.EnergyAxes, sprintf('Energy per Robot  (Total=%.0f)',sum(totalE)), ...
                'Color',[0.88 0.88 0.93],'FontSize',9);
            legend(app.EnergyAxes,{'Move','Push'}, ...
                'TextColor',[0.80 0.80 0.85],'Color',[0.14 0.14 0.17],...
                'EdgeColor',[0.30 0.30 0.35],'FontSize',7,'Location','northwest');

            % ── Progress lines ────────────────────────────────────────────
            cla(app.ProgressAxes);
            hold(app.ProgressAxes,'on');
            for i = 1:n
                eh = app.sim.robots{i}.energyHistory;
                if numel(eh)>1
                    plot(app.ProgressAxes, eh, '-', ...
                        'Color', colors(i,:), 'LineWidth', 1.8);
                end
            end
            hold(app.ProgressAxes,'off');
            app.ProgressAxes.Color    = [0.13 0.13 0.15];
            app.ProgressAxes.XColor   = [0.65 0.65 0.65];
            app.ProgressAxes.YColor   = [0.65 0.65 0.65];
            app.ProgressAxes.YGrid    = 'on';
            app.ProgressAxes.GridColor= [0.28 0.28 0.28];
            title(app.ProgressAxes,'Energy Over Time',...
                'Color',[0.88 0.88 0.93],'FontSize',9);
            xlabel(app.ProgressAxes,'Step',...
                'Color',[0.65 0.65 0.65],'FontSize',8);

            % ── Status panel ──────────────────────────────────────────────
            cla(app.StatusAxes);
            app.StatusAxes.XLim = [0 1];
            app.StatusAxes.YLim = [0 1];

            nDone  = sum(cellfun(@(r) r.isDone(),    app.sim.robots));
            nStuck = sum(cellfun(@(r) r.isStuck(),   app.sim.robots));
            nWait  = sum(cellfun(@(r) r.isWaiting(), app.sim.robots));
            nMove  = sum(cellfun(@(r) r.isMoving(),  app.sim.robots));

            lines = {
                sprintf('Step      %d', app.sim.stepCount)
                sprintf('Done      %d / %d', nDone, n)
                sprintf('Moving    %d', nMove)
                sprintf('Waiting   %d', nWait)
                sprintf('Stuck     %d', nStuck)
                sprintf('Removed   %d obstacles', numel(app.sim.removalPlan))
                sprintf('Total E   %.1f', sum(totalE))
            };
            cols2 = [0.70 0.80 1.00;
                     0.40 0.95 0.50;
                     0.50 0.80 1.00;
                     0.95 0.90 0.30;
                     1.00 0.40 0.40;
                     0.80 0.65 1.00;
                     1.00 0.70 0.30];

            for li = 1:numel(lines)
                text(app.StatusAxes, 0.04, 1 - li*0.135, lines{li}, ...
                    'Color', cols2(li,:), 'FontSize', 10, ...
                    'FontName', 'Courier New', 'FontWeight', 'bold', ...
                    'Units', 'normalized', 'Interpreter', 'none');
            end
            app.StatusAxes.XAxis.Visible = 'off';
            app.StatusAxes.YAxis.Visible = 'off';

            drawnow limitrate;
        end

        function clearAxes(app)
            cla(app.GridAxes);
            cla(app.EnergyAxes);
            cla(app.ProgressAxes);
            cla(app.StatusAxes);
            title(app.GridAxes, 'MAPF-RO Environment', ...
                'Color',[0.85 0.85 0.90],'FontSize',11,'FontWeight','bold');
            drawnow;
        end

        function setStatus(app, msg, color)
            if nargin < 3, color = [0.65 0.75 0.65]; end
            if isvalid(app.LblStatus)
                app.LblStatus.Text      = ['  ' msg];
                app.LblStatus.FontColor = color;
            end
        end

        function gs = getGridSizeStr(app)
            raw = app.DropGridSize.Value;
            if contains(raw,'small'),  gs = 'small';
            elseif contains(raw,'medium'), gs = 'medium';
            else,  gs = 'large';
            end
        end

        function cols = getRobotColors(~)
            cols = [
                0.20 0.60 1.00;
                1.00 0.40 0.10;
                0.20 0.85 0.45;
                0.90 0.20 0.30;
                0.80 0.20 0.90;
                0.95 0.85 0.10;
                0.10 0.85 0.85;
                1.00 0.55 0.70;
                0.60 0.90 0.30;
                0.40 0.30 0.90;
            ];
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  APP LAUNCH
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = public)
        function app = MAPFRO_App()
            app.startupFcn();
            if nargout == 0
                clear app;
            end
        end
    end
end