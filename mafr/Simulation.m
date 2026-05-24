classdef Simulation < handle
    % SIMULATION - Master controller for MAPF-RO
    %
    % Orchestrates the full simulation loop:
    %   1. Initialise environment, robots, planner, obstacle manager
    %   2. Run Algorithm 1 (overall planning)
    %   3. Execute robots step-by-step with live animation
    %   4. Collect metrics: energy, success rate, path length
    %   5. Plot paper results (Figures 4, 5, 6)
    %
    % Usage:
    %   sim = Simulation('size','small','nRobots',5,'k',2);
    %   sim.run();           % full animated run
    %   sim.runBatch();      % reproduce paper Fig 4,5,6

    %% ── Configuration ───────────────────────────────────────────────────
    properties
        % Environment
        gridSize    (1,:) char   = 'small'   % 'small'|'medium'|'large'
        nRobots     (1,1) double = 5
        pushK       (1,1) double = 2         % sandbag push multiplier
        pitRatio    (1,2) double = [1 1]     % pit:sandbag ratio
        seed        (1,1) double = 0         % 0 = random seed

        % Animation
        animSpeed   (1,1) double = 0.15      % seconds per step
        showAnim    (1,1) logical = true
        showTraffic (1,1) logical = false

        % Simulation mode
        useRO       (1,1) logical = true      % true=MAPF-RO, false=baseline
    end

    %% ── Runtime objects ─────────────────────────────────────────────────
    properties (Access = public)
        env         Environment
        robots      cell
        planner     AStarPlanner
        manager     ObstacleManager

        initPaths   cell
        finalPaths  cell
        removalPlan struct

        % Metrics collected after run
        metrics     struct
    end

    %% ── Figure handles ──────────────────────────────────────────────────
    properties (Access = private)
        fig         % main figure
        axGrid      % grid axes (left panel)
        axEnergy    % energy bar chart (right top)
        axStatus    % status text panel (right bottom)
        axProgress  % live energy progress (right mid)

        % Graphic handles updated each frame
        hRobots     cell    % scatter handles for robots
        hPaths      cell    % line handles for paths
        hEnergies   % bar handle

        isRunning   (1,1) logical = false
        isPaused    (1,1) logical = false
    end

    %% ── Step counter (public so App can read it) ─────────────────────
    properties (Access = public)
        stepCount   (1,1) double = 0
    end

    %% ════════════════════════════════════════════════════════════════════
    %  CONSTRUCTOR
    %% ════════════════════════════════════════════════════════════════════
    methods
        function obj = Simulation(varargin)
            p = inputParser;
            addParameter(p, 'size',       'small', @ischar);
            addParameter(p, 'nRobots',    5,       @isnumeric);
            addParameter(p, 'k',          2,       @isnumeric);
            addParameter(p, 'ratio',      [1 1],   @isnumeric);
            addParameter(p, 'seed',       0,       @isnumeric);
            addParameter(p, 'speed',      0.15,    @isnumeric);
            addParameter(p, 'animate',    true,    @islogical);
            addParameter(p, 'useRO',      true,    @islogical);
            parse(p, varargin{:});

            obj.gridSize  = p.Results.size;
            obj.nRobots   = p.Results.nRobots;
            obj.pushK     = p.Results.k;
            obj.pitRatio  = p.Results.ratio;
            obj.seed      = p.Results.seed;
            obj.animSpeed = p.Results.speed;
            obj.showAnim  = p.Results.animate;
            obj.useRO     = p.Results.useRO;

            obj.metrics = struct();
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  PUBLIC: MAIN RUN
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = public)

        function run(obj)
            % Full simulation: initialise → plan → animate → results

            obj.initialise();
            obj.plan();

            if obj.showAnim
                obj.setupFigure();
                obj.animate();
            end

            obj.collectMetrics();
            obj.printSummary();
        end

        function initialise(obj)
            % Step 1: Build environment and robots
            if obj.seed > 0
                rng(obj.seed);
            end

            switch lower(obj.gridSize)
                case 'small'
                    obj.env = Environment.small(obj.nRobots, obj.pitRatio);
                case 'medium'
                    obj.env = Environment.medium(obj.nRobots, obj.pitRatio);
                case 'large'
                    obj.env = Environment.large(obj.nRobots, obj.pitRatio);
                otherwise
                    error('Simulation: unknown size "%s"', obj.gridSize);
            end

            obj.robots  = Robot.fromEnvironment(obj.env, obj.pushK);
            obj.planner = AStarPlanner(obj.env);
            obj.manager = ObstacleManager(obj.env, obj.planner);
        end

        function plan(obj)
            % Step 2: Run planning pipeline (Algorithm 1)
            obj.initPaths = obj.planner.planAllRobots(obj.robots);

            if obj.useRO
                [obj.finalPaths, obj.removalPlan] = ...
                    obj.manager.overallPlanning(obj.robots, obj.initPaths);
            else
                % Baseline: treat all obstacles as static
                obj.finalPaths  = obj.planner.planAllRobots(obj.robots, ...
                    'treatPitsAsStatic', true, ...
                    'treatSandbagsAsStatic', true);
                obj.removalPlan = struct('type',{},'pos',{},'robotId',{},...
                    'sandbagPos',{},'cost',{});
            end

            % Assign final paths to robots
            for i = 1:numel(obj.robots)
                if ~isempty(obj.finalPaths{i})
                    obj.robots{i}.setPath(obj.finalPaths{i});
                else
                    obj.robots{i}.markStuck();
                end
            end
        end

        function animate(obj)
            % Step 3: Step robots through their paths with live rendering
            obj.isRunning = true;
            obj.stepCount = 0;

            maxSteps = obj.env.rows * obj.env.cols * 3;

            while obj.isRunning && obj.stepCount < maxSteps
                % Check if all robots finished
                allDone = all(cellfun(@(r) r.isFinished(), obj.robots));
                if allDone, break; end

                % Advance each robot one step
                occupied = obj.getCurrentPositions();
                for i = 1:numel(obj.robots)
                    r = obj.robots{i};
                    if r.isFinished(), continue; end

                    % Collision check: if next cell occupied → wait
                    nextPos = obj.getNextPos(r);
                    if ~isempty(nextPos)
                        others = occupied;
                        others(i,:) = [];
                        if any(others(:,1)==nextPos(1) & others(:,2)==nextPos(2))
                            r.pause();
                        else
                            r.resume();
                        end
                    end

                    % Execute step
                    obj.executeRobotStep(r, i);
                end

                obj.stepCount = obj.stepCount + 1;

                % Render frame
                if obj.showAnim && isvalid(obj.fig)
                    obj.renderFrame();
                    pause(obj.animSpeed);
                end
            end

            obj.isRunning = false;

            % Final render
            if obj.showAnim && isvalid(obj.fig)
                obj.renderFrame();
                obj.updateStatus('Simulation complete!', [0.2 0.9 0.4]);
            end
        end

        function metrics = collectMetrics(obj)
            % Collect final metrics from all robots
            n = numel(obj.robots);

            energies    = zeros(1, n);
            moveEnergy  = zeros(1, n);
            pushEnergy  = zeros(1, n);
            success     = false(1, n);
            pathLengths = zeros(1, n);

            for i = 1:n
                r = obj.robots{i};
                energies(i)    = r.energyTotal;
                moveEnergy(i)  = r.energyMove;
                pushEnergy(i)  = r.energyPush;
                success(i)     = r.isDone();
                pathLengths(i) = size(r.travelledPath, 1);
            end

            obj.metrics.totalEnergy   = sum(energies);
            obj.metrics.avgEnergy     = mean(energies);
            obj.metrics.perRobotEnergy = energies;
            obj.metrics.moveEnergy    = moveEnergy;
            obj.metrics.pushEnergy    = pushEnergy;
            obj.metrics.successCount  = sum(success);
            obj.metrics.successRate   = mean(success);
            obj.metrics.pathLengths   = pathLengths;
            obj.metrics.nRobots       = n;
            obj.metrics.steps         = obj.stepCount;
            obj.metrics.removals      = numel(obj.removalPlan);
            obj.metrics.useRO         = obj.useRO;

            metrics = obj.metrics;
        end

        function printSummary(obj)
            m = obj.metrics;
            fprintf('\n╔══════════════════════════════════════════════╗\n');
            fprintf('║         SIMULATION RESULTS                   ║\n');
            fprintf('╠══════════════════════════════════════════════╣\n');
            if m.useRO
                modeStr = 'MAPF-RO';
            else
                modeStr = 'Baseline (no RO)';
            end
            fprintf('║  Mode        : %s\n', modeStr);
            fprintf('║  Robots      : %d\n',   m.nRobots);
            fprintf('║  Success     : %d/%d (%.0f%%)\n', ...
                m.successCount, m.nRobots, m.successRate*100);
            fprintf('║  Total Energy: %.1f\n',  m.totalEnergy);
            fprintf('║  Avg Energy  : %.1f\n',  m.avgEnergy);
            fprintf('║  Removals    : %d obstacles\n', m.removals);
            fprintf('║  Steps taken : %d\n',   m.steps);
            fprintf('╚══════════════════════════════════════════════╝\n\n');
        end

        % ── Batch experiment (reproduce paper Figures 4, 5, 6) ──────────
        function runBatch(obj, varargin)
            % runBatch() — runs across robot counts and k values
            % Reproduces paper experiment: 15 environments, 5-25 robots

            p = inputParser;
            addParameter(p, 'robotCounts', [1 2 3 4 5 6 7 8 9 10], @isnumeric);
            addParameter(p, 'kValues',     [1 2 3 4],               @isnumeric);
            addParameter(p, 'nEnvs',       5,                       @isnumeric);
            addParameter(p, 'gridSize',    'small',                  @ischar);
            parse(p, varargin{:});

            robotCounts = p.Results.robotCounts;
            kValues     = p.Results.kValues;
            nEnvs       = p.Results.nEnvs;
            gs          = p.Results.gridSize;

            fprintf('\nRunning batch experiment...\n');
            fprintf('Grid: %s | Envs: %d | Robots: %s | k values: %s\n\n', ...
                gs, nEnvs, mat2str(robotCounts), mat2str(kValues));

            % Storage: [nRobots x nK x nEnvs]
            energyRO   = zeros(numel(robotCounts), numel(kValues), nEnvs);
            energyBase = zeros(numel(robotCounts), 1,              nEnvs);
            successRO  = zeros(numel(robotCounts), 1,              nEnvs);
            successBase= zeros(numel(robotCounts), 1,              nEnvs);

            for ei = 1:nEnvs
                seed = ei * 100;
                fprintf('  Environment %d/%d...\n', ei, nEnvs);

                for ri = 1:numel(robotCounts)
                    nR = robotCounts(ri);

                    % ── Baseline (MAPF without RO) ──────────────────────
                    simBase = Simulation('size',gs,'nRobots',nR,...
                        'k',2,'seed',seed,'animate',false,'useRO',false);
                    simBase.initialise();
                    simBase.plan();
                    simBase.animate();
                    mBase = simBase.collectMetrics();
                    energyBase(ri,1,ei) = mBase.totalEnergy;
                    successBase(ri,1,ei)= mBase.successRate;

                    % ── MAPF-RO for each k ───────────────────────────────
                    for ki = 1:numel(kValues)
                        k = kValues(ki);
                        simRO = Simulation('size',gs,'nRobots',nR,...
                            'k',k,'seed',seed,'animate',false,'useRO',true);
                        simRO.initialise();
                        simRO.plan();
                        simRO.animate();
                        mRO = simRO.collectMetrics();
                        energyRO(ri,ki,ei) = mRO.totalEnergy;
                        if ki == 1
                            successRO(ri,1,ei) = mRO.successRate;
                        end
                    end
                end
            end

            % ── Average across environments ──────────────────────────────
            avgEnergyRO   = mean(energyRO,   3);   % [nRobots x nK]
            avgEnergyBase = mean(energyBase, 3);   % [nRobots x 1]
            avgSuccessRO  = mean(successRO,  3);
            avgSuccessBase= mean(successBase,3);

            % ── Plot results ─────────────────────────────────────────────
            obj.plotBatchResults(robotCounts, kValues, ...
                avgEnergyRO, avgEnergyBase, ...
                avgSuccessRO, avgSuccessBase);
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  PRIVATE: FIGURE SETUP & RENDERING
    %% ════════════════════════════════════════════════════════════════════
    %% ── Public helpers called by MAPFRO_App ────────────────────────────
    methods (Access = public)

        function pos = getCurrentPositions(obj)
            n = numel(obj.robots);
            pos = zeros(n, 2);
            for i = 1:n
                pos(i,:) = obj.robots{i}.currentPos;
            end
        end

        function nextPos = getNextPos(obj, robot)
            nextPos = [];
            if robot.isFinished() || isempty(robot.plannedPath), return; end
            nextStep = robot.pathStep + 1;
            if nextStep <= size(robot.plannedPath, 1)
                nextPos = robot.plannedPath(nextStep, :);
            end
        end

        function executeRobotStep(obj, robot, ~)
            if robot.isFinished() || robot.isWaiting()
                robot.stepForward(obj.env); return;
            end
            if robot.hasTask && ~robot.taskComplete
                sb = robot.assignedSandbag;
                pt = robot.assignedPit;
                if isequal(robot.currentPos, sb) || ...
                   (robot.distanceTo(sb) <= 1 && obj.env.isSandbag(sb(1),sb(2)))
                    if obj.env.isSandbag(sb(1),sb(2)) && obj.env.isPit(pt(1),pt(2))
                        obj.env.fillPit(pt(1), pt(2));
                        obj.env.removeSandbag(sb(1), sb(2));
                        robot.addPushCost(robot.distanceTo(pt));
                        robot.completeTask();
                    else
                        robot.completeTask();
                    end
                end
            end
            robot.stepForward(obj.env);
        end

    end

    methods (Access = private)

        function setupFigure(obj)
            % Create the main simulation window (Nav2-style layout)
            obj.fig = figure(...
                'Name',        'MAPF-RO Live Simulation', ...
                'Color',       [0.10 0.10 0.12], ...
                'NumberTitle', 'off', ...
                'Position',    [50 50 1200 700], ...
                'CloseRequestFcn', @(~,~) obj.onClose());

            % ── Left panel: grid (large) ─────────────────────────────────
            obj.axGrid = axes('Parent', obj.fig, ...
                'Position', [0.02 0.05 0.60 0.90], ...
                'Color',    [0.13 0.13 0.15]);

            % ── Right panel: energy bars ─────────────────────────────────
            obj.axEnergy = axes('Parent', obj.fig, ...
                'Position', [0.66 0.55 0.32 0.38], ...
                'Color',    [0.13 0.13 0.15], ...
                'XColor',   [0.7 0.7 0.7], ...
                'YColor',   [0.7 0.7 0.7]);

            % ── Right panel: live energy progress ────────────────────────
            obj.axProgress = axes('Parent', obj.fig, ...
                'Position', [0.66 0.30 0.32 0.20], ...
                'Color',    [0.13 0.13 0.15], ...
                'XColor',   [0.7 0.7 0.7], ...
                'YColor',   [0.7 0.7 0.7]);

            % ── Right panel: status ──────────────────────────────────────
            obj.axStatus = axes('Parent', obj.fig, ...
                'Position', [0.66 0.05 0.32 0.22], ...
                'Color',    [0.10 0.10 0.12], ...
                'Visible',  'off');

            % Draw initial state
            obj.renderFrame();
        end

        function renderFrame(obj)
            % Redraw the grid and all overlays for current sim state

            % ── Grid ─────────────────────────────────────────────────────
            cla(obj.axGrid);
            obj.env.visualize('ax', obj.axGrid, ...
                'traffic', obj.showTraffic, ...
                'title', sprintf('MAPF-RO  |  Step %d  |  %s', ...
                    obj.stepCount, upper(obj.gridSize)));

            % ── Draw planned paths (faded) ───────────────────────────────
            hold(obj.axGrid, 'on');
            colors = obj.getRobotColors();
            for i = 1:numel(obj.robots)
                r = obj.robots{i};
                if isempty(r.plannedPath), continue; end
                col = colors(i,:);

                % Full planned path (very faint)
                pth = r.plannedPath;
                if size(pth,1) > 1
                    plot(obj.axGrid, pth(:,2), pth(:,1), '--', ...
                        'Color', [col, 0.20], 'LineWidth', 1.0);
                end

                % Travelled path (solid, brighter)
                tp = r.travelledPath;
                if size(tp,1) > 1
                    plot(obj.axGrid, tp(:,2), tp(:,1), '-', ...
                        'Color', [col, 0.60], 'LineWidth', 2.0);
                end

                % Goal marker
                scatter(obj.axGrid, r.goalPos(2), r.goalPos(1), 140, 'd', ...
                    'MarkerFaceColor', col*0.6+0.3, ...
                    'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
                text(obj.axGrid, r.goalPos(2), r.goalPos(1), ...
                    sprintf('G%d',i), 'Color','w','FontSize',7,...
                    'FontWeight','bold','HorizontalAlignment','center',...
                    'VerticalAlignment','middle');

                % Robot body
                markerSize = 200;
                if r.isDone()
                    % Done: star shape, greyed
                    scatter(obj.axGrid, r.currentPos(2), r.currentPos(1), ...
                        markerSize, 'p', ...
                        'MarkerFaceColor', col*0.5+0.3, ...
                        'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
                elseif r.isStuck()
                    scatter(obj.axGrid, r.currentPos(2), r.currentPos(1), ...
                        markerSize, 'x', ...
                        'MarkerEdgeColor', [1 0.2 0.2], 'LineWidth', 3);
                elseif r.isWaiting()
                    scatter(obj.axGrid, r.currentPos(2), r.currentPos(1), ...
                        markerSize, 's', ...
                        'MarkerFaceColor', col, ...
                        'MarkerEdgeColor', [1 1 0], 'LineWidth', 2.5);
                else
                    scatter(obj.axGrid, r.currentPos(2), r.currentPos(1), ...
                        markerSize, 'o', ...
                        'MarkerFaceColor', col, ...
                        'MarkerEdgeColor', 'w', 'LineWidth', 2.0);
                end

                % Robot label
                text(obj.axGrid, r.currentPos(2), r.currentPos(1), ...
                    sprintf('R%d',i), 'Color','w','FontSize',7,...
                    'FontWeight','bold','HorizontalAlignment','center',...
                    'VerticalAlignment','middle');
            end

            % ── Removal plan markers ─────────────────────────────────────
            for ri = 1:numel(obj.removalPlan)
                rp = obj.removalPlan(ri);
                if strcmp(rp.type,'pit')
                    % Arrow from sandbag to pit
                    plot(obj.axGrid, ...
                        [rp.sandbagPos(2), rp.pos(2)], ...
                        [rp.sandbagPos(1), rp.pos(1)], ...
                        '->', 'Color', [1 1 0.3 0.8], 'LineWidth', 1.5, ...
                        'MarkerSize', 6);
                end
            end
            hold(obj.axGrid, 'off');

            % ── Energy bar chart ─────────────────────────────────────────
            cla(obj.axEnergy);
            n = numel(obj.robots);
            energies = cellfun(@(r) r.energyTotal, obj.robots);
            moveE    = cellfun(@(r) r.energyMove,  obj.robots);
            pushE    = cellfun(@(r) r.energyPush,  obj.robots);

            barData = [moveE; pushE]';
            b = bar(obj.axEnergy, barData, 'stacked');
            b(1).FaceColor = [0.25 0.60 1.00];
            b(1).EdgeColor = 'none';
            if numel(b) > 1
                b(2).FaceColor = [1.00 0.45 0.10];
                b(2).EdgeColor = 'none';
            end

            obj.axEnergy.XTickLabel = arrayfun(@(i) sprintf('R%d',i), 1:n, ...
                'UniformOutput', false);
            obj.axEnergy.Color    = [0.13 0.13 0.15];
            obj.axEnergy.XColor   = [0.7 0.7 0.7];
            obj.axEnergy.YColor   = [0.7 0.7 0.7];
            obj.axEnergy.GridColor= [0.3 0.3 0.3];
            obj.axEnergy.YGrid    = 'on';
            title(obj.axEnergy, ...
                sprintf('Energy per Robot  (Total=%.0f)', sum(energies)), ...
                'Color',[0.9 0.9 0.9],'FontSize',9);
            ylabel(obj.axEnergy,'Energy','Color',[0.7 0.7 0.7],'FontSize',8);
            legend(obj.axEnergy, {'Move','Push'}, ...
                'TextColor',[0.8 0.8 0.8],'Color',[0.15 0.15 0.18],...
                'EdgeColor',[0.3 0.3 0.3],'FontSize',7,'Location','northwest');

            % ── Live energy progress lines ────────────────────────────────
            cla(obj.axProgress);
            hold(obj.axProgress, 'on');
            cols = obj.getRobotColors();
            for i = 1:numel(obj.robots)
                eh = obj.robots{i}.energyHistory;
                if numel(eh) > 1
                    plot(obj.axProgress, eh, '-', ...
                        'Color', cols(i,:), 'LineWidth', 1.5);
                end
            end
            hold(obj.axProgress, 'off');
            obj.axProgress.Color    = [0.13 0.13 0.15];
            obj.axProgress.XColor   = [0.6 0.6 0.6];
            obj.axProgress.YColor   = [0.6 0.6 0.6];
            obj.axProgress.YGrid    = 'on';
            obj.axProgress.GridColor= [0.25 0.25 0.25];
            title(obj.axProgress,'Energy Over Time', ...
                'Color',[0.9 0.9 0.9],'FontSize',9);
            xlabel(obj.axProgress,'Step','Color',[0.6 0.6 0.6],'FontSize',7);

            % ── Status panel ─────────────────────────────────────────────
            cla(obj.axStatus);
            nDone   = sum(cellfun(@(r) r.isDone(),    obj.robots));
            nStuck  = sum(cellfun(@(r) r.isStuck(),   obj.robots));
            nWait   = sum(cellfun(@(r) r.isWaiting(), obj.robots));
            nMove   = sum(cellfun(@(r) r.isMoving(),  obj.robots));

            statusLines = {
                sprintf('Step:    %d', obj.stepCount),
                sprintf('Done:    %d/%d', nDone, numel(obj.robots)),
                sprintf('Moving:  %d', nMove),
                sprintf('Waiting: %d', nWait),
                sprintf('Stuck:   %d', nStuck),
                sprintf('Removed: %d obstacles', numel(obj.removalPlan)),
            };

            for li = 1:numel(statusLines)
                text(obj.axStatus, 0.05, 1 - li*0.15, statusLines{li}, ...
                    'Color', [0.85 0.85 0.85], 'FontSize', 9, ...
                    'FontName', 'Monospaced', 'Units', 'normalized');
            end
            obj.axStatus.Visible = 'off';

            drawnow limitrate;
        end

        function updateStatus(obj, msg, color)
            if nargin < 3, color = [0.9 0.9 0.9]; end
            text(obj.axStatus, 0.5, 0.05, msg, ...
                'Color', color, 'FontSize', 11, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', 'Units', 'normalized');
            drawnow;
        end




        function cols = getRobotColors(obj)
            palette = [
                0.20 0.60 1.00;
                1.00 0.40 0.10;
                0.20 0.85 0.45;
                0.90 0.20 0.30;
                0.80 0.20 0.90;
                0.95 0.85 0.10;
                0.10 0.85 0.85;
                1.00 0.55 0.70;
            ];
            n = numel(obj.robots);
            cols = palette(mod((1:n)-1, size(palette,1))+1, :);
        end

        function onClose(obj)
            obj.isRunning = false;
            delete(obj.fig);
        end

        function plotBatchResults(obj, robotCounts, kValues, ...
                avgEnergyRO, avgEnergyBase, avgSuccessRO, avgSuccessBase)
            % Plot paper Figures 4, 5, 6

            fig2 = figure('Name','MAPF-RO Batch Results', ...
                'Color',[0.10 0.10 0.12], 'Position',[100 100 1100 380], ...
                'NumberTitle','off');

            kColors = [0.25 0.75 1.0;
                       0.25 1.00 0.5;
                       1.00 0.70 0.1;
                       1.00 0.30 0.3];

            % ── Fig 4: Total energy vs robots (per k) ────────────────────
            ax4 = subplot(1,3,1,'Parent',fig2);
            hold(ax4,'on');
            for ki = 1:numel(kValues)
                plot(ax4, robotCounts, avgEnergyRO(:,ki), '-o', ...
                    'Color', kColors(ki,:), 'LineWidth', 2, ...
                    'MarkerSize', 5, 'MarkerFaceColor', kColors(ki,:), ...
                    'DisplayName', sprintf('k=%d', kValues(ki)));
            end
            hold(ax4,'off');
            obj.styleAxis(ax4, 'No. of Robots', 'Total Energy', ...
                'Fig 4: Total Energy (MAPF-RO)', kValues, 'k=');

            % ── Fig 5: Success rate RO vs Baseline ───────────────────────
            ax5 = subplot(1,3,2,'Parent',fig2);
            hold(ax5,'on');
            plot(ax5, robotCounts, avgSuccessRO,   '-o', ...
                'Color',[0.25 0.85 0.45],'LineWidth',2.5,'MarkerSize',5,...
                'MarkerFaceColor',[0.25 0.85 0.45],'DisplayName','MAPF-RO');
            plot(ax5, robotCounts, avgSuccessBase, '--s', ...
                'Color',[0.50 0.50 1.00],'LineWidth',2.5,'MarkerSize',5,...
                'MarkerFaceColor',[0.50 0.50 1.00],'DisplayName','MAPF w/o RO');
            hold(ax5,'off');
            ylim(ax5,[0 1.05]);
            obj.styleAxis(ax5, 'No. of Robots', 'Success Rate', ...
                'Fig 5: Success Rate', [], '');

            % ── Fig 6: Avg energy RO vs Baseline per environment ─────────
            ax6 = subplot(1,3,3,'Parent',fig2);
            nR  = numel(robotCounts);
            bar(ax6, [avgEnergyBase(:,1), avgEnergyRO(:,1)], ...
                'grouped');
            ax6.Children(1).FaceColor = [0.25 0.85 0.45];
            ax6.Children(2).FaceColor = [0.50 0.50 1.00];
            obj.styleAxis(ax6, 'Robot Count Index', 'Avg Energy', ...
                'Fig 6: Avg Energy vs Baseline', [], '');
            legend(ax6, {'MAPF w/o RO','MAPF-RO'}, ...
                'TextColor',[0.8 0.8 0.8],'Color',[0.15 0.15 0.18], ...
                'EdgeColor',[0.3 0.3 0.3],'FontSize',8);
        end

        function styleAxis(~, ax, xl, yl, ttl, legendVals, legendPfx)
            ax.Color    = [0.13 0.13 0.15];
            ax.XColor   = [0.7 0.7 0.7];
            ax.YColor   = [0.7 0.7 0.7];
            ax.GridColor= [0.3 0.3 0.3];
            ax.XGrid    = 'on';
            ax.YGrid    = 'on';
            xlabel(ax, xl,  'Color',[0.7 0.7 0.7],'FontSize',9);
            ylabel(ax, yl,  'Color',[0.7 0.7 0.7],'FontSize',9);
            title(ax,  ttl, 'Color',[0.9 0.9 0.9],'FontSize',10,'FontWeight','bold');
            if ~isempty(legendVals)
                legend(ax, 'TextColor',[0.8 0.8 0.8],'Color',[0.15 0.15 0.18],...
                    'EdgeColor',[0.3 0.3 0.3],'FontSize',8,'Location','northwest');
            end
        end
    end
end