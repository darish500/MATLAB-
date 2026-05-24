classdef AStarPlanner < handle
    % ASTARPLANNER - Weighted A* path planner for MAPF-RO
    %
    % Implements the weighted cost function from the paper (Eq. 1):
    %
    %   g(n) = g(n-1) + w(n)
    %
    % where w(n) = 1 for free cells, and higher penalty values for
    % cells containing pits or sandbags, to discourage but not
    % completely forbid crossing them.
    %
    % Static obstacles are NEVER entered (infinite cost = blocked).
    %
    % Features:
    %   - 4-connected grid movement (paper assumption)
    %   - Manhattan distance heuristic
    %   - Traffic-index weighting (busy nodes cost more)
    %   - Obstacle penalty tuning via properties
    %   - Returns [n x 2] path of [row, col] waypoints
    %   - Returns cost=Inf if no path exists
    %
    % Usage:
    %   planner = AStarPlanner(env);
    %   [path, cost] = planner.plan(startPos, goalPos);
    %   [path, cost] = planner.plan(startPos, goalPos, 'treatPitsAsStatic', true);

    %% ── Tunable parameters ──────────────────────────────────────────────
    properties
        % Penalty w(n) for each cell type (paper: "higher values for
        % nodes with obstacles").  Static = Inf (never entered).
        penaltyFree     (1,1) double = 1      % w for FREE cell
        penaltyPit      (1,1) double = 2      % w for PIT: just 1 extra step cost
        penaltySandbag  (1,1) double = 2      % w for SANDBAG: same
        penaltyStatic   (1,1) double = Inf    % w for STATIC (blocked)

        % Traffic index influence weight
        % Final cell cost = baseCost + trafficWeight * trafficIndex(r,c)
        trafficWeight   (1,1) double = 0.2

        % If true, pits are treated as static (used in MAPF without RO
        % baseline comparison — paper Section V.3)
        treatPitsAsStatic    (1,1) logical = false
        treatSandbagsAsStatic(1,1) logical = false
    end

    %% ── Internal ────────────────────────────────────────────────────────
    properties (Access = private)
        env     Environment    % reference to the environment
    end

    %% ════════════════════════════════════════════════════════════════════
    %  CONSTRUCTOR
    %% ════════════════════════════════════════════════════════════════════
    methods
        function obj = AStarPlanner(env)
            arguments
                env Environment
            end
            obj.env = env;
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  PUBLIC: MAIN PLANNING METHOD
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = public)

        function [path, cost] = plan(obj, startPos, goalPos, varargin)
            % [path, cost] = plan(startPos, goalPos)
            % [path, cost] = plan(startPos, goalPos, Name, Value, ...)
            %
            % startPos : [row col]
            % goalPos  : [row col]
            %
            % Optional Name-Value:
            %   'treatPitsAsStatic'     true/false  (override property)
            %   'treatSandbagsAsStatic' true/false
            %   'occupiedCells'         [m x 2]  cells blocked by other robots
            %
            % Returns:
            %   path : [n x 2] of [row col] from start to goal (inclusive)
            %          Empty [] if no path found
            %   cost : scalar total g-cost of path, Inf if no path

            p = inputParser;
            addParameter(p, 'treatPitsAsStatic',     obj.treatPitsAsStatic,     @islogical);
            addParameter(p, 'treatSandbagsAsStatic', obj.treatSandbagsAsStatic, @islogical);
            addParameter(p, 'occupiedCells',         zeros(0,2),                @isnumeric);
            parse(p, varargin{:});

            pitsStatic  = p.Results.treatPitsAsStatic;
            sbsStatic   = p.Results.treatSandbagsAsStatic;
            occupied    = p.Results.occupiedCells;

            % ── Validate inputs ──────────────────────────────────────────
            if ~obj.env.inBounds(startPos(1), startPos(2))
                error('AStarPlanner: start %s is out of bounds.', mat2str(startPos));
            end
            if ~obj.env.inBounds(goalPos(1), goalPos(2))
                error('AStarPlanner: goal %s is out of bounds.', mat2str(goalPos));
            end
            % Silently return no path if start/goal is static
            % (can happen during temporary obstacle simulation in detour cost calc)
            if obj.env.isStatic(startPos(1), startPos(2))
                path = []; cost = Inf; return;
            end
            if obj.env.isStatic(goalPos(1), goalPos(2))
                path = []; cost = Inf; return;
            end

            % ── Trivial case ─────────────────────────────────────────────
            if isequal(startPos, goalPos)
                path = startPos;
                cost = 0;
                return;
            end

            % ── A* data structures ───────────────────────────────────────
            rows = obj.env.rows;
            cols = obj.env.cols;

            % g-cost matrix (Inf = not yet visited)
            gCost = inf(rows, cols);
            gCost(startPos(1), startPos(2)) = 0;

            % f-cost = g + h
            fCost = inf(rows, cols);
            fCost(startPos(1), startPos(2)) = obj.heuristic(startPos, goalPos);

            % Parent tracking for path reconstruction
            parentRow = zeros(rows, cols, 'int16');
            parentCol = zeros(rows, cols, 'int16');

            % Open set: store as matrix [f, r, c]
            % We use a simple sorted list (good enough for grids ≤ 40x40)
            openSet = [fCost(startPos(1),startPos(2)), startPos(1), startPos(2)];
            closedSet = false(rows, cols);

            found = false;

            % ── Main A* loop ─────────────────────────────────────────────
            while ~isempty(openSet)

                % Pop node with lowest f-cost
                [~, idx] = min(openSet(:,1));
                current  = openSet(idx, 2:3);
                openSet(idx, :) = [];   % remove from open set

                cr = current(1);
                cc = current(2);

                % Skip if already closed
                if closedSet(cr, cc)
                    continue;
                end
                closedSet(cr, cc) = true;

                % ── Goal check ───────────────────────────────────────────
                if cr == goalPos(1) && cc == goalPos(2)
                    found = true;
                    break;
                end

                % ── Expand neighbours ────────────────────────────────────
                nbrs = obj.env.getNeighbours(cr, cc);

                for ni = 1:size(nbrs, 1)
                    nr = nbrs(ni, 1);
                    nc = nbrs(ni, 2);

                    if closedSet(nr, nc)
                        continue;
                    end

                    % Check if blocked by another robot
                    if ~isempty(occupied)
                        if any(occupied(:,1)==nr & occupied(:,2)==nc)
                            continue;
                        end
                    end

                    % Compute cell cost w(n)
                    w = obj.cellCost(nr, nc, pitsStatic, sbsStatic);

                    if isinf(w)
                        continue;   % impassable
                    end

                    % Add traffic index contribution
                    tIdx = obj.env.trafficIndex(nr, nc);
                    w = w + obj.trafficWeight * tIdx;

                    tentativeG = gCost(cr, cc) + w;

                    if tentativeG < gCost(nr, nc)
                        gCost(nr, nc) = tentativeG;
                        parentRow(nr, nc) = int16(cr);
                        parentCol(nr, nc) = int16(cc);

                        f = tentativeG + obj.heuristic([nr nc], goalPos);
                        fCost(nr, nc) = f;

                        % Add to open set
                        openSet(end+1, :) = [f, nr, nc]; %#ok<AGROW>
                    end
                end
            end

            % ── Reconstruct path ─────────────────────────────────────────
            if found
                path = obj.reconstructPath(parentRow, parentCol, startPos, goalPos);
                cost = gCost(goalPos(1), goalPos(2));
            else
                path = [];
                cost = Inf;
            end
        end

        function [path, cost] = planBaseline(obj, startPos, goalPos)
            % Plan treating ALL obstacles as static
            % Used for "MAPF without RO" baseline comparison (paper Fig 6)
            [path, cost] = obj.plan(startPos, goalPos, ...
                'treatPitsAsStatic', true, ...
                'treatSandbagsAsStatic', true);
        end

        function detourCost = computeDetourCost(obj, originalPath, obstaclePos)
            % Compute average detour cost if obstacle at obstaclePos
            % is made temporarily static.
            % Used in Algorithm 1 lines 13-21 of paper.
            %
            % Returns: extra cost vs original path length

            if isempty(originalPath)
                detourCost = Inf;
                return;
            end

            % Check if obstacle is on this path
            onPath = any(originalPath(:,1) == obstaclePos(1) & ...
                         originalPath(:,2) == obstaclePos(2));

            if ~onPath
                detourCost = 0;
                return;
            end

            % Temporarily make obstacle static and replan
            originalType = obj.env.grid(obstaclePos(1), obstaclePos(2));
            obj.env.grid(obstaclePos(1), obstaclePos(2)) = obj.env.STATIC;

            start = originalPath(1, :);
            goal  = originalPath(end, :);
            [detourPath, detourG] = obj.plan(start, goal, ...
                'treatPitsAsStatic', true, 'treatSandbagsAsStatic', true);

            % Restore
            obj.env.grid(obstaclePos(1), obstaclePos(2)) = originalType;

            if isempty(detourPath)
                detourCost = Inf;
            else
                originalCost = size(originalPath,1) - 1;  % approx step count
                detourCost   = detourG - originalCost;
                detourCost   = max(detourCost, 0);
            end
        end

        function updateEnvironment(obj, env)
            % Swap the environment reference (used after grid changes)
            obj.env = env;
        end

        function paths = planAllRobots(obj, robots, varargin)
            % Plan paths for all robots, passing occupied starts as blocked
            % Returns cell array of paths {robot1path, robot2path, ...}
            %
            % Simple priority: plan in order robot 1,2,...n
            % Each robot's START is added to occupied list for subsequent robots

            p = inputParser;
            addParameter(p, 'treatPitsAsStatic',     false, @islogical);
            addParameter(p, 'treatSandbagsAsStatic',  false, @islogical);
            parse(p, varargin{:});

            n = numel(robots);
            paths = cell(1, n);
            occupied = zeros(0, 2);

            for i = 1:n
                r = robots{i};
                [pth, ~] = obj.plan(r.currentPos, r.goalPos, ...
                    'treatPitsAsStatic',      p.Results.treatPitsAsStatic, ...
                    'treatSandbagsAsStatic',  p.Results.treatSandbagsAsStatic, ...
                    'occupiedCells',          occupied);

                paths{i} = pth;

                % Add this robot's current position to occupied
                occupied(end+1, :) = r.currentPos; %#ok<AGROW>
            end
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  PRIVATE HELPERS
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = private)

        function h = heuristic(~, pos, goal)
            % Manhattan distance (admissible for 4-connected grid)
            h = abs(pos(1) - goal(1)) + abs(pos(2) - goal(2));
        end

        function w = cellCost(obj, r, c, pitsStatic, sbsStatic)
            % Returns the w(n) cost of entering cell (r,c)
            % Paper eq. 1: w(n)=1 for navigable, higher for obstacles

            cellType = obj.env.grid(r, c);

            switch cellType
                case obj.env.FREE
                    w = obj.penaltyFree;

                case obj.env.STATIC
                    w = Inf;

                case obj.env.PIT
                    if pitsStatic
                        w = Inf;
                    else
                        w = obj.penaltyPit;
                    end

                case obj.env.SANDBAG
                    if sbsStatic
                        w = Inf;
                    else
                        w = obj.penaltySandbag;
                    end

                otherwise
                    w = obj.penaltyFree;
            end
        end

        function path = reconstructPath(~, parentRow, parentCol, startPos, goalPos)
            % Walk backwards from goal to start using parent pointers
            path = goalPos;
            r = goalPos(1);
            c = goalPos(2);

            maxIter = size(parentRow,1) * size(parentRow,2);
            iter = 0;

            while ~(r == startPos(1) && c == startPos(2))
                pr = double(parentRow(r, c));
                pc = double(parentCol(r, c));

                if pr == 0 && pc == 0
                    % No parent → path reconstruction failed
                    path = [];
                    return;
                end

                path = [pr, pc; path]; %#ok<AGROW>
                r = pr;
                c = pc;

                iter = iter + 1;
                if iter > maxIter
                    warning('AStarPlanner: path reconstruction exceeded max iterations.');
                    path = [];
                    return;
                end
            end
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  VISUALISATION HELPER
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = public)

        function visualizePlan(obj, paths, robots, ax)
            % Overlay planned paths on an existing axes
            % paths : cell array of [n x 2] paths
            % robots: cell array of Robot objects
            % ax    : axes handle

            if nargin < 4 || isempty(ax)
                ax = gca;
            end

            hold(ax, 'on');

            colors = [
                0.20 0.60 1.00;
                1.00 0.40 0.10;
                0.20 0.85 0.45;
                0.90 0.20 0.30;
                0.80 0.20 0.90;
                0.95 0.85 0.10;
                0.10 0.85 0.85;
                1.00 0.55 0.70;
            ];

            for i = 1:numel(paths)
                pth = paths{i};
                if isempty(pth), continue; end

                col = colors(mod(i-1, size(colors,1))+1, :);

                % Draw path line
                plot(ax, pth(:,2), pth(:,1), '-', ...
                    'Color', [col, 0.7], 'LineWidth', 2.0);

                % Start marker
                scatter(ax, pth(1,2), pth(1,1), 140, 'o', ...
                    'MarkerFaceColor', col, 'MarkerEdgeColor', 'w', 'LineWidth',1.5);

                % Goal marker
                scatter(ax, pth(end,2), pth(end,1), 140, 'd', ...
                    'MarkerFaceColor', col*0.7+0.3, 'MarkerEdgeColor', 'w', 'LineWidth',1.5);

                % Labels
                text(ax, pth(1,2), pth(1,1), sprintf('R%d',i), ...
                    'Color','w','FontSize',7,'FontWeight','bold',...
                    'HorizontalAlignment','center','VerticalAlignment','middle');
                text(ax, pth(end,2), pth(end,1), sprintf('G%d',i), ...
                    'Color','w','FontSize',7,'FontWeight','bold',...
                    'HorizontalAlignment','center','VerticalAlignment','middle');
            end

            hold(ax, 'off');
        end
    end
end