classdef Robot < handle
    % ROBOT - Represents a single robot in the MAPF-RO simulation
    %
    % Each robot has:
    %   - An ID, start position, goal position
    %   - A planned path (sequence of [row,col] waypoints)
    %   - An energy counter (accumulated cost)
    %   - A state machine (idle, moving, pushing, waiting, done)
    %   - A task assignment (which pit to fill, which sandbag to push)
    %
    % Energy model (from paper Section III):
    %   - Moving one cell costs:        c = 1
    %   - Pushing a sandbag one cell:   k*c  where k > 1
    %   - Waiting (idle):               ~0  (negligible, per paper)
    %
    % Usage:
    %   r = Robot(1, [3 4], [18 17], env);
    %   r.info()

    %% ── State enumeration ───────────────────────────────────────────────
    % We use string states for readability
    % Valid states:
    %   'idle'     → has no path yet, waiting for planner
    %   'moving'   → following its planned path normally
    %   'detouring'→ deviating from main path to fetch a sandbag
    %   'pushing'  → actively pushing a sandbag toward a pit
    %   'waiting'  → paused to let another robot pass (collision avoidance)
    %   'done'     → reached goal successfully
    %   'stuck'    → no valid path exists (failure)

    properties (Constant)
        MOVE_COST   = 1     % cost c to move one cell (paper eq. 1)
        DEFAULT_K   = 2     % default push multiplier k (paper Section III)
    end

    %% ── Identity ────────────────────────────────────────────────────────
    properties
        id          (1,1) double       % robot index (1-based)
        startPos    (1,2) double       % [row col] start
        goalPos     (1,2) double       % [row col] goal
        currentPos  (1,2) double       % [row col] current position
        color       (1,3) double       % RGB colour for visualisation
    end

    %% ── Path & navigation ───────────────────────────────────────────────
    properties
        plannedPath  (:,2) double      % full planned path [n x 2]
        travelledPath(:,2) double      % path already executed [n x 2]
        pathStep     (1,1) double = 1  % index into plannedPath
        detourPath   (:,2) double      % temporary detour to sandbag
    end

    %% ── Energy accounting ───────────────────────────────────────────────
    properties
        energyTotal   (1,1) double = 0  % total energy spent so far
        energyMove    (1,1) double = 0  % energy from plain movement
        energyPush    (1,1) double = 0  % energy from pushing sandbags
        pushK         (1,1) double      % push multiplier k (per paper)
    end

    %% ── State machine ───────────────────────────────────────────────────
    properties
        state       (1,:) char = 'idle'   % current state string
        waitCount   (1,1) double = 0      % steps spent waiting
    end

    %% ── Task assignment (set by ObstacleManager) ────────────────────────
    properties
        assignedPit      (1,2) double = [0 0]   % [row col] of pit to fill
        assignedSandbag  (1,2) double = [0 0]   % [row col] of sandbag to push
        hasTask          (1,1) logical = false   % true if assigned a pit task
        taskComplete     (1,1) logical = false   % true when pit is filled
    end

    %% ── History (for plotting / analysis) ───────────────────────────────
    properties
        energyHistory (:,1) double   % energy at each time step
        stateHistory  (:,1) cell     % state at each time step
        posHistory    (:,2) double   % position at each time step
    end

    %% ════════════════════════════════════════════════════════════════════
    %  CONSTRUCTOR
    %% ════════════════════════════════════════════════════════════════════
    methods
        function obj = Robot(id, startPos, goalPos, pushK)
            % Robot(id, startPos, goalPos)
            % Robot(id, startPos, goalPos, pushK)
            %
            % id       : integer robot ID
            % startPos : [row col]
            % goalPos  : [row col]
            % pushK    : energy multiplier for pushing (default = 2)

            arguments
                id       (1,1) double
                startPos (1,2) double
                goalPos  (1,2) double
                pushK    (1,1) double = Robot.DEFAULT_K
            end

            obj.id         = id;
            obj.startPos   = startPos;
            obj.goalPos    = goalPos;
            obj.currentPos = startPos;
            obj.pushK      = pushK;

            % Assign a distinct colour from a palette
            palette = [
                0.20 0.60 1.00;   % blue
                1.00 0.40 0.10;   % orange
                0.20 0.85 0.45;   % green
                0.90 0.20 0.30;   % red
                0.80 0.20 0.90;   % purple
                0.95 0.85 0.10;   % yellow
                0.10 0.85 0.85;   % cyan
                1.00 0.55 0.70;   % pink
            ];
            obj.color = palette(mod(id-1, size(palette,1)) + 1, :);

            % Initialise history
            obj.energyHistory = 0;
            obj.stateHistory  = {'idle'};
            obj.posHistory    = startPos;
            obj.travelledPath = startPos;
            obj.plannedPath   = zeros(0,2);
            obj.detourPath    = zeros(0,2);
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  PATH & MOVEMENT
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = public)

        function setPath(obj, path)
            % Assign a new planned path and reset step counter
            % path: [n x 2] matrix of [row col] waypoints
            if isempty(path)
                warning('Robot %d: setPath called with empty path.', obj.id);
                return;
            end
            obj.plannedPath = path;
            obj.pathStep    = 1;
            obj.setState('moving');
        end

        function setDetourPath(obj, path)
            % Assign a detour path (robot leaving main path to fetch sandbag)
            obj.detourPath = path;
            obj.setState('detouring');
        end

        function success = stepForward(obj, env)
            % Move robot one step along its current path.
            % Returns true if step was taken, false if blocked/done.
            %
            % env: Environment object (used to check cell types)

            success = false;

            switch obj.state
                case 'done'
                    return;

                case 'stuck'
                    return;

                case 'waiting'
                    obj.waitCount = obj.waitCount + 1;
                    obj.recordHistory();
                    success = true;
                    return;

                case 'moving'
                    success = obj.takeStep(obj.plannedPath, env, false);

                case 'detouring'
                    if ~isempty(obj.detourPath)
                        success = obj.takeStep(obj.detourPath, env, false);
                    end

                case 'pushing'
                    success = obj.takeStep(obj.plannedPath, env, true);

                case 'idle'
                    % Nothing to do yet
                    obj.recordHistory();
                    return;
            end
        end

        function tf = hasReachedGoal(obj)
            tf = isequal(obj.currentPos, obj.goalPos);
        end

        function tf = isFinished(obj)
            tf = strcmp(obj.state, 'done') || strcmp(obj.state, 'stuck');
        end

        function remainingSteps = stepsRemaining(obj)
            if isempty(obj.plannedPath)
                remainingSteps = 0;
            else
                remainingSteps = size(obj.plannedPath,1) - obj.pathStep;
            end
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  STATE MANAGEMENT
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = public)

        function setState(obj, newState)
            validStates = {'idle','moving','detouring','pushing','waiting','done','stuck'};
            if ~ismember(newState, validStates)
                error('Robot %d: invalid state "%s"', obj.id, newState);
            end
            obj.state = newState;
        end

        function pause(obj)
            % Make robot wait (collision avoidance)
            if ~obj.isFinished()
                obj.setState('waiting');
            end
        end

        function resume(obj)
            % Resume movement after waiting
            if strcmp(obj.state, 'waiting')
                if ~isempty(obj.detourPath) && obj.pathStep <= size(obj.detourPath,1)
                    obj.setState('detouring');
                else
                    obj.setState('moving');
                end
            end
        end

        function markDone(obj)
            obj.setState('done');
        end

        function markStuck(obj)
            obj.setState('stuck');
        end

        function assignTask(obj, pitPos, sandbagPos)
            % Assign a pit-filling task to this robot
            obj.assignedPit     = pitPos;
            obj.assignedSandbag = sandbagPos;
            obj.hasTask         = true;
            obj.taskComplete    = false;
        end

        function completeTask(obj)
            obj.taskComplete = true;
            obj.hasTask      = false;
            % Return to moving state after task
            obj.setState('moving');
        end

        function clearTask(obj)
            obj.assignedPit     = [0 0];
            obj.assignedSandbag = [0 0];
            obj.hasTask         = false;
            obj.taskComplete    = false;
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  ENERGY ACCOUNTING
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = public)

        function addMoveCost(obj, nSteps)
            % Add plain movement energy  (c per step)
            if nargin < 2, nSteps = 1; end
            cost = nSteps * obj.MOVE_COST;
            obj.energyMove  = obj.energyMove  + cost;
            obj.energyTotal = obj.energyTotal + cost;
        end

        function addPushCost(obj, nSteps)
            % Add push energy  (k*c per step, paper Section III)
            if nargin < 2, nSteps = 1; end
            cost = nSteps * obj.pushK * obj.MOVE_COST;
            obj.energyPush  = obj.energyPush  + cost;
            obj.energyTotal = obj.energyTotal + cost;
        end

        function resetEnergy(obj)
            obj.energyTotal = 0;
            obj.energyMove  = 0;
            obj.energyPush  = 0;
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  QUERY HELPERS
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = public)

        function d = distanceTo(obj, pos)
            % Manhattan distance from current position to pos [r c]
            d = abs(obj.currentPos(1) - pos(1)) + ...
                abs(obj.currentPos(2) - pos(2));
        end

        function d = distanceToGoal(obj)
            d = obj.distanceTo(obj.goalPos);
        end

        function tf = isAt(obj, pos)
            tf = isequal(obj.currentPos, pos);
        end

        function tf = isIdle(obj)
            tf = strcmp(obj.state, 'idle');
        end

        function tf = isMoving(obj)
            tf = ismember(obj.state, {'moving','detouring','pushing'});
        end

        function tf = isWaiting(obj)
            tf = strcmp(obj.state, 'waiting');
        end

        function tf = isDone(obj)
            tf = strcmp(obj.state, 'done');
        end

        function tf = isStuck(obj)
            tf = strcmp(obj.state, 'stuck');
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  DISPLAY & DEBUG
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = public)

        function info(obj)
            fprintf('\n── Robot %d ─────────────────────────────────\n', obj.id);
            fprintf('  State      : %s\n',   obj.state);
            fprintf('  Position   : [%d, %d]\n', obj.currentPos(1), obj.currentPos(2));
            fprintf('  Start      : [%d, %d]\n', obj.startPos(1),   obj.startPos(2));
            fprintf('  Goal       : [%d, %d]\n', obj.goalPos(1),    obj.goalPos(2));
            fprintf('  Dist2Goal  : %d steps\n', obj.distanceToGoal());
            fprintf('  Path steps : %d planned, at step %d\n', ...
                size(obj.plannedPath,1), obj.pathStep);
            fprintf('  Energy     : %.1f total  (move=%.1f, push=%.1f)\n', ...
                obj.energyTotal, obj.energyMove, obj.energyPush);
            fprintf('  Push k     : %.1f\n', obj.pushK);
            fprintf('  Has task   : %s\n',   mat2str(obj.hasTask));
            if obj.hasTask
                fprintf('  → Pit      : [%d,%d]\n', obj.assignedPit(1), obj.assignedPit(2));
                fprintf('  → Sandbag  : [%d,%d]\n', obj.assignedSandbag(1), obj.assignedSandbag(2));
            end
            fprintf('─────────────────────────────────────────────\n\n');
        end

        function drawOnAxes(obj, ax, varargin)
            % Draw this robot's current position and path on an axes
            % Optional: 'showPath', true/false
            %           'showTrail', true/false

            p = inputParser;
            addParameter(p, 'showPath',  true,  @islogical);
            addParameter(p, 'showTrail', true,  @islogical);
            addParameter(p, 'markerSize', 180,  @isnumeric);
            parse(p, varargin{:});

            hold(ax, 'on');

            % ── Travelled trail (faded) ───────────────────────────────
            if p.Results.showTrail && size(obj.travelledPath,1) > 1
                plot(ax, obj.travelledPath(:,2), obj.travelledPath(:,1), ...
                    '-', 'Color', [obj.color, 0.25], 'LineWidth', 1.5);
            end

            % ── Planned path ahead (dashed) ───────────────────────────
            if p.Results.showPath && ~isempty(obj.plannedPath)
                remaining = obj.plannedPath(obj.pathStep:end, :);
                if size(remaining,1) > 1
                    plot(ax, remaining(:,2), remaining(:,1), '--', ...
                        'Color', [obj.color, 0.55], 'LineWidth', 1.2);
                end
            end

            % ── Robot body (filled circle) ────────────────────────────
            scatter(ax, obj.currentPos(2), obj.currentPos(1), ...
                p.Results.markerSize, 'o', ...
                'MarkerFaceColor', obj.color, ...
                'MarkerEdgeColor', [1 1 1], ...
                'LineWidth', 2.0);

            % ── Robot ID label ────────────────────────────────────────
            text(ax, obj.currentPos(2), obj.currentPos(1), ...
                sprintf('R%d', obj.id), ...
                'Color', 'white', 'FontSize', 7, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');

            % ── Goal marker ───────────────────────────────────────────
            scatter(ax, obj.goalPos(2), obj.goalPos(1), ...
                p.Results.markerSize, 'd', ...
                'MarkerFaceColor', obj.color * 0.7 + [0.3 0.3 0.3], ...
                'MarkerEdgeColor', [1 1 1], ...
                'LineWidth', 1.5);

            text(ax, obj.goalPos(2), obj.goalPos(1), ...
                sprintf('G%d', obj.id), ...
                'Color', 'white', 'FontSize', 7, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');

            hold(ax, 'off');
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  PRIVATE HELPERS
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = private)

        function success = takeStep(obj, path, env, isPushing)
            % Execute one step along 'path'
            % isPushing: true → apply push energy cost

            success = false;
            if isempty(path) || obj.pathStep >= size(path,1)
                % Reached end of this path segment
                if obj.hasReachedGoal()
                    obj.markDone();
                end
                return;
            end

            nextStep = obj.pathStep + 1;
            nextPos  = path(nextStep, :);

            % Safety check — don't step into static obstacles
            if env.isStatic(nextPos(1), nextPos(2))
                obj.markStuck();
                return;
            end

            % Apply energy cost
            if isPushing
                obj.addPushCost(1);
            else
                obj.addMoveCost(1);
            end

            % Move
            obj.currentPos = nextPos;
            obj.pathStep   = nextStep;
            obj.travelledPath(end+1, :) = nextPos;

            % Check goal
            if obj.hasReachedGoal()
                obj.markDone();
            end

            obj.recordHistory();
            success = true;
        end

        function recordHistory(obj)
            obj.energyHistory(end+1) = obj.energyTotal;
            obj.stateHistory{end+1}  = obj.state;
            obj.posHistory(end+1, :) = obj.currentPos;
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  STATIC FACTORY
    %% ════════════════════════════════════════════════════════════════════
    methods (Static)
        function robots = fromEnvironment(env, pushK)
            % Create all robots from an Environment object
            % Returns: cell array of Robot objects
            arguments
                env   Environment
                pushK (1,1) double = Robot.DEFAULT_K
            end

            n = env.nRobots;
            robots = cell(1, n);
            for i = 1:n
                robots{i} = Robot(i, env.robotStarts(i,:), ...
                                     env.robotGoals(i,:), pushK);
            end
        end
    end
end