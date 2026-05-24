classdef ObstacleManager < handle
    % OBSTACLEMANAGER - Implements Algorithms 1 & 2 from the paper
    %
    % Algorithm 1: Overall_Planning
    %   - Finds obstacles on robot paths
    %   - Compares removal cost vs detour cost
    %   - Decides which obstacles to remove
    %   - Assigns robots to tasks
    %
    % Algorithm 2: Cost of Filling Pit
    %   - Uses kd-tree to find m nearest sandbags to a pit
    %   - Uses directional wavefront propagation to find best robot
    %   - Computes min cost triple: (robot, sandbag, pit)
    %
    % Paper reference: Section IV-B, Algorithms 1 & 2

    %% ── Parameters ──────────────────────────────────────────────────────
    properties
        % Number of nearest sandbags to consider (m in paper)
        % m ∝ 1/k  (more candidates when push is cheap)
        mCandidates     (1,1) double = 5

        % Directional wavefront bias weights (paper eq. 3)
        % w(x',y') bias: lower = preferred direction (toward pit)
        biasToward      (1,1) double = 1.0   % toward pit direction
        biasAway        (1,1) double = 4.0   % away from pit (penalised)
        biasSide        (1,1) double = 2.0   % perpendicular

        % Removal cost threshold multiplier
        % If removalCost > threshold * detourCost → keep obstacle
        removalThreshold (1,1) double = 1.0

        % Traffic index threshold: only remove obstacles with tv >= this
        trafficThreshold (1,1) double = 1
    end

    %% ── Internal references ─────────────────────────────────────────────
    properties (Access = private)
        env     Environment
        planner AStarPlanner

        % Assignment log for display
        assignments     (:,1) struct   % array of assignment records
    end

    %% ════════════════════════════════════════════════════════════════════
    %  CONSTRUCTOR
    %% ════════════════════════════════════════════════════════════════════
    methods
        function obj = ObstacleManager(env, planner)
            arguments
                env     Environment
                planner AStarPlanner
            end
            obj.env     = env;
            obj.planner = planner;
            obj.assignments = struct('pitPos',{},'sandbagPos',{},...
                                     'robotId',{},'cost',{});
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  PUBLIC: ALGORITHM 1 — Overall Planning
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = public)

        function [updatedPaths, removalPlan] = overallPlanning(obj, robots, initialPaths)
            % Implements Algorithm 1 from paper (Overall_Planning)
            %
            % Inputs:
            %   robots       : cell array of Robot objects
            %   initialPaths : cell array of initial A* paths
            %
            % Outputs:
            %   updatedPaths : cell array of final paths (after removals)
            %   removalPlan  : struct array describing what gets removed

            updatedPaths = initialPaths;
            removalPlan  = struct('type',{},'pos',{},'robotId',{},...
                                  'sandbagPos',{},'cost',{});

            % ── Step 1: Build traffic index ──────────────────────────────
            obj.env.resetTraffic();
            for i = 1:numel(initialPaths)
                if ~isempty(initialPaths{i})
                    obj.env.incrementTraffic(initialPaths{i});
                end
            end

            % ── Step 2: Find all obstacles on paths ──────────────────────
            obstacles = obj.getObstaclesOnPaths(initialPaths);

            if isempty(obstacles)
                return;   % no obstacles to deal with
            end

            % ── Step 3: Sort by traffic index (highest first) ────────────
            tvVals = arrayfun(@(o) obj.env.trafficIndex(o.pos(1),o.pos(2)), obstacles);
            [~, sortIdx] = sort(tvVals, 'descend');
            obstacles = obstacles(sortIdx);

            % ── Step 4: For each obstacle, decide remove vs detour ───────
            for oi = 1:numel(obstacles)
                obs = obstacles(oi);

                % Skip low-traffic obstacles (paper: prioritise busy nodes)
                tv = obj.env.trafficIndex(obs.pos(1), obs.pos(2));
                if tv < obj.trafficThreshold
                    continue;
                end

                % Find which robots have this obstacle on their path
                affectedRobots = obs.robotIds;
                lv = numel(affectedRobots);  % traffic index = number of paths

                % Compute removal cost
                if strcmp(obs.type, 'pit')
                    [co, assignedRobotId, sandbagPos] = ...
                        obj.costOfFillingPit(obs.pos, robots, initialPaths);
                else
                    % Sandbag: cost of moving it away
                    [co, assignedRobotId, sandbagPos] = ...
                        obj.costOfMovingSandbag(obs.pos, robots, initialPaths);
                end

                % Compute average detour cost for affected robots
                totalDetour = 0;
                newPathsTemp = initialPaths;
                for ri = 1:numel(affectedRobots)
                    rIdx = affectedRobots(ri);
                    detour = obj.planner.computeDetourCost(...
                        initialPaths{rIdx}, obs.pos);
                    totalDetour = totalDetour + detour;

                    % Store detour path
                    [detourPath, ~] = obj.planner.plan(...
                        robots{rIdx}.currentPos, robots{rIdx}.goalPos, ...
                        'treatPitsAsStatic',     strcmp(obs.type,'pit'), ...
                        'treatSandbagsAsStatic',  strcmp(obs.type,'sandbag'));
                    newPathsTemp{rIdx} = detourPath;
                end
                avgDetour = totalDetour / max(lv, 1);

                % ── Decision (Algorithm 1, lines 21-34) ──────────────────
                if isinf(co)
                    % Cannot remove → make static, replan
                    obj.env.setCellType(obs.pos(1), obs.pos(2), obj.env.STATIC);
                    for ri = 1:numel(affectedRobots)
                        rIdx = affectedRobots(ri);
                        [rp, ~] = obj.planner.plan(...
                            robots{rIdx}.currentPos, robots{rIdx}.goalPos);
                        updatedPaths{rIdx} = rp;
                    end

                elseif avgDetour < co * obj.removalThreshold
                    % Detour is cheaper → keep obstacle, use detour paths
                    for ri = 1:numel(affectedRobots)
                        rIdx = affectedRobots(ri);
                        updatedPaths{rIdx} = newPathsTemp{rIdx};
                    end

                else
                    % Removal is worth it → assign robot to remove
                    if assignedRobotId > 0
                        % Assign task to robot
                        sp = sandbagPos;
                        if strcmp(obs.type, 'pit')
                            robots{assignedRobotId}.assignTask(obs.pos, sp);
                        else
                            robots{assignedRobotId}.assignTask(obs.pos, obs.pos);
                        end

                        % Log the removal plan
                        entry.type      = obs.type;
                        entry.pos       = obs.pos;
                        entry.robotId   = assignedRobotId;
                        entry.sandbagPos = sp;
                        entry.cost      = co;
                        removalPlan(end+1) = entry; %#ok<AGROW>

                        % Replan all affected robots after removal
                        % (simulate removal then replan)
                        origType = obj.env.grid(obs.pos(1), obs.pos(2));
                        obj.env.grid(obs.pos(1), obs.pos(2)) = obj.env.FREE;

                        for ri = 1:numel(affectedRobots)
                            rIdx = affectedRobots(ri);
                            [rp, ~] = obj.planner.plan(...
                                robots{rIdx}.currentPos, robots{rIdx}.goalPos);
                            updatedPaths{rIdx} = rp;
                        end

                        % Restore for now (actual removal happens during sim)
                        obj.env.grid(obs.pos(1), obs.pos(2)) = origType;
                    end
                end
            end

            % ── Rebuild traffic index with final paths ───────────────────
            obj.env.resetTraffic();
            for i = 1:numel(updatedPaths)
                if ~isempty(updatedPaths{i})
                    obj.env.incrementTraffic(updatedPaths{i});
                end
            end
        end

        function [cost, robotId, sandbagPos] = costOfFillingPit(obj, pitPos, robots, paths)
            % Algorithm 2: Cost of Filling Pit
            %
            % 1. Use kd-tree to find m nearest sandbags
            % 2. For each sandbag, use directional wavefront to find
            %    nearest robot path
            % 3. Return minimum total cost triple

            cost      = Inf;
            robotId   = 0;
            sandbagPos = [0 0];

            sbPos = obj.env.sandbagPositions;
            if isempty(sbPos)
                return;
            end

            % ── Step 1: kd-tree to find m nearest sandbags ───────────────
            % kd-tree ignores obstacles (heuristic, per paper)
            m = min(obj.mCandidates, size(sbPos,1));
            nearestSBs = obj.kdTreeSearch(pitPos, sbPos, m);

            % ── Step 2: For each candidate sandbag ───────────────────────
            for si = 1:size(nearestSBs, 1)
                sb = nearestSBs(si, :);

                % Actual A* distance: sandbag → pit
                [sbToPitPath, sbToPitCost] = obj.planner.plan(sb, pitPos);
                if isempty(sbToPitPath) || isinf(sbToPitCost)
                    continue;
                end

                % ── Step 3: Directional wavefront from sandbag ────────────
                % Find robot whose path passes nearest to sandbag,
                % traveling in direction of pit
                pitDir = obj.getDirection(sb, pitPos);
                waveMap = obj.directionalWavefront(sb, pitDir);

                bestRobotId   = 0;
                bestTotal     = Inf;
                bestRobotPoint = [0 0];

                for ri = 1:numel(robots)
                    if isempty(paths{ri}), continue; end

                    % Find point on robot path closest to sandbag
                    [robotPoint, distToPath] = obj.nearestPointOnPath(...
                        paths{ri}, sb, waveMap);

                    if isinf(distToPath), continue; end

                    % path_len = extra distance robot travels for task
                    % Cost = (k+1) * path_len  (paper Algorithm 2, line 8)
                    % k * path_len to push + path_len to return
                    k = robots{ri}.pushK;
                    pathLen = distToPath + sbToPitCost;
                    total   = (k + 1) * pathLen;

                    if total < bestTotal
                        bestTotal      = total;
                        bestRobotId    = ri;
                        bestRobotPoint = robotPoint;
                    end
                end

                if bestTotal < cost
                    cost       = bestTotal;
                    robotId    = bestRobotId;
                    sandbagPos = sb;
                end
            end
        end

        function [cost, robotId, movedTo] = costOfMovingSandbag(obj, sbPos, robots, paths)
            % Cost of moving a sandbag out of the path
            % Uses BFS to find nearest free cell, then wavefront for robot

            cost    = Inf;
            robotId = 0;
            movedTo = [0 0];

            % BFS to find nearest free cell to move sandbag to
            freeTarget = obj.bfsNearestFree(sbPos);
            if isempty(freeTarget)
                return;
            end

            moveDir = obj.getDirection(sbPos, freeTarget);
            waveMap = obj.directionalWavefront(sbPos, moveDir);

            [~, sbToFreeCost] = obj.planner.plan(sbPos, freeTarget);
            if isinf(sbToFreeCost), return; end

            for ri = 1:numel(robots)
                if isempty(paths{ri}), continue; end

                [~, distToPath] = obj.nearestPointOnPath(paths{ri}, sbPos, waveMap);
                if isinf(distToPath), continue; end

                k = robots{ri}.pushK;
                total = (k + 1) * (distToPath + sbToFreeCost);

                if total < cost
                    cost    = total;
                    robotId = ri;
                    movedTo = freeTarget;
                end
            end
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  PRIVATE: CORE ALGORITHMS
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = private)

        function nearest = kdTreeSearch(obj, queryPos, candidates, m)
            % Simplified kd-tree: Euclidean distance sort (ignores obstacles)
            % Paper: "kd-tree serves as a heuristic search pointing to
            %         nearest sandbags ignoring obstacles"
            %
            % For MATLAB we use pdist2 which is equivalent for this use

            if isempty(candidates)
                nearest = zeros(0, 2);
                return;
            end

            dists = pdist2(double(queryPos), double(candidates));
            [~, idx] = sort(dists);
            m = min(m, numel(idx));
            nearest = candidates(idx(1:m), :);
        end

        function waveMap = directionalWavefront(obj, origin, pitDir)
            % Directional wavefront propagation (paper eq. 2 & 3)
            %
            % Propagates from 'origin' with bias AWAY from pitDir
            % (so waves spread toward robots coming FROM the pit direction)
            %
            % pitDir: [dr dc] unit direction vector toward pit
            %
            % Returns: waveMap [rows x cols] — cost from origin to each cell

            rows = obj.env.rows;
            cols = obj.env.cols;

            waveMap = inf(rows, cols);
            waveMap(origin(1), origin(2)) = 0;

            % BFS queue: [r c cost]
            queue = [origin(1), origin(2), 0];
            head  = 1;

            while head <= size(queue, 1)
                cr   = queue(head, 1);
                cc   = queue(head, 2);
                curr = queue(head, 3);
                head = head + 1;

                % 4-connected neighbours
                moves = [-1 0; 1 0; 0 -1; 0 1];

                for mi = 1:4
                    nr = cr + moves(mi, 1);
                    nc = cc + moves(mi, 2);

                    if ~obj.env.inBounds(nr, nc), continue; end
                    if obj.env.isStatic(nr, nc),  continue; end

                    % Directional bias weight w(x',y') — paper eq. 3
                    moveDir = [moves(mi,1), moves(mi,2)];
                    bias    = obj.directionalBias(moveDir, pitDir);

                    newCost = curr + bias;

                    if newCost < waveMap(nr, nc)
                        waveMap(nr, nc) = newCost;
                        queue(end+1, :) = [nr, nc, newCost]; %#ok<AGROW>
                    end
                end
            end
        end

        function bias = directionalBias(obj, moveDir, pitDir)
            % Compute bias weight for a move direction (paper eq. 3)
            % w(x',y') = w * C(x',y')
            %
            % We direct waves AWAY from pitDir (toward robots coming from that side)
            % So: moving AWAY from pit = low cost (preferred)
            %     moving TOWARD pit    = high cost (discouraged)

            dot = moveDir(1)*pitDir(1) + moveDir(2)*pitDir(2);
            % dot =  1: moving toward pit
            % dot = -1: moving away from pit (preferred for wave)
            % dot =  0: perpendicular

            if dot > 0
                bias = obj.biasAway;      % moving toward pit = penalised
            elseif dot < 0
                bias = obj.biasToward;    % moving away from pit = preferred
            else
                bias = obj.biasSide;      % perpendicular
            end
        end

        function dir = getDirection(~, fromPos, toPos)
            % Get cardinal direction vector [dr dc] from fromPos to toPos
            dr = toPos(1) - fromPos(1);
            dc = toPos(2) - fromPos(2);

            % Normalise to dominant axis (4-connected movement)
            if abs(dr) >= abs(dc)
                dir = [sign(dr), 0];
            else
                dir = [0, sign(dc)];
            end

            if all(dir == 0)
                dir = [0, 1];  % default
            end
        end

        function [bestPoint, minDist] = nearestPointOnPath(~, path, origin, waveMap)
            % Find point on 'path' with minimum wavefront cost from origin
            % Also returns the actual distance to that point

            bestPoint = [0 0];
            minDist   = Inf;

            for pi = 1:size(path, 1)
                r = path(pi, 1);
                c = path(pi, 2);

                if r >= 1 && r <= size(waveMap,1) && ...
                   c >= 1 && c <= size(waveMap,2)
                    d = waveMap(r, c);
                    if d < minDist
                        minDist   = d;
                        bestPoint = [r c];
                    end
                end
            end
        end

        function freePos = bfsNearestFree(obj, origin)
            % BFS to find nearest free cell (for moving sandbags out of path)
            % Free = not a sandbag, pit, static, or robot goal

            rows = obj.env.rows;
            cols = obj.env.cols;
            visited = false(rows, cols);
            visited(origin(1), origin(2)) = true;

            queue = [origin(1), origin(2)];
            head  = 1;

            while head <= size(queue, 1)
                cr = queue(head, 1);
                cc = queue(head, 2);
                head = head + 1;

                % Check if this is a free cell (and not the origin)
                if obj.env.isFree(cr, cc) && ~isequal([cr cc], origin)
                    freePos = [cr cc];
                    return;
                end

                moves = [-1 0; 1 0; 0 -1; 0 1];
                for mi = 1:4
                    nr = cr + moves(mi,1);
                    nc = cc + moves(mi,2);

                    if obj.env.inBounds(nr,nc) && ~visited(nr,nc) && ...
                       ~obj.env.isStatic(nr,nc)
                        visited(nr,nc) = true;
                        queue(end+1,:) = [nr nc]; %#ok<AGROW>
                    end
                end
            end

            freePos = [];  % no free cell found
        end

        function obstacles = getObstaclesOnPaths(obj, paths)
            % Find all pits and sandbags that appear on any robot path
            % Returns struct array with fields: type, pos, robotIds

            obsMap = containers.Map('KeyType','char','ValueType','any');

            for ri = 1:numel(paths)
                pth = paths{ri};
                if isempty(pth), continue; end

                for pi = 1:size(pth, 1)
                    r = pth(pi, 1);
                    c = pth(pi, 2);
                    ct = obj.env.grid(r, c);

                    if ct == obj.env.PIT || ct == obj.env.SANDBAG
                        key = sprintf('%d_%d', r, c);
                        if obsMap.isKey(key)
                            entry = obsMap(key);
                            if ~ismember(ri, entry.robotIds)
                                entry.robotIds(end+1) = ri;
                                obsMap(key) = entry;
                            end
                        else
                            if ct == obj.env.PIT
                                tp = 'pit';
                            else
                                tp = 'sandbag';
                            end
                            entry = struct('type', tp, 'pos', [r c], ...
                                           'robotIds', ri);
                            obsMap(key) = entry;
                        end
                    end
                end
            end

            % Convert map to struct array
            keys = obsMap.keys();
            obstacles = struct('type',{},'pos',{},'robotIds',{});
            for ki = 1:numel(keys)
                obstacles(end+1) = obsMap(keys{ki}); %#ok<AGROW>
            end
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  PUBLIC: DISPLAY & REPORTING
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = public)

        function printRemovalPlan(~, removalPlan)
            if isempty(removalPlan)
                fprintf('\n  [ObstacleManager] No obstacles scheduled for removal.\n\n');
                return;
            end
            fprintf('\n══════════════════════════════════════════════════\n');
            fprintf('  OBSTACLE REMOVAL PLAN  (%d assignments)\n', numel(removalPlan));
            fprintf('══════════════════════════════════════════════════\n');
            for i = 1:numel(removalPlan)
                rp = removalPlan(i);
                if strcmp(rp.type, 'pit')
                    fprintf('  [%d] PIT     at [%d,%d] ← Sandbag [%d,%d]  Robot R%d  cost=%.1f\n', ...
                        i, rp.pos(1), rp.pos(2), ...
                        rp.sandbagPos(1), rp.sandbagPos(2), ...
                        rp.robotId, rp.cost);
                else
                    fprintf('  [%d] SANDBAG at [%d,%d] → moved elsewhere   Robot R%d  cost=%.1f\n', ...
                        i, rp.pos(1), rp.pos(2), rp.robotId, rp.cost);
                end
            end
            fprintf('══════════════════════════════════════════════════\n\n');
        end

        function visualizeTraffic(obj, ax)
            % Overlay traffic heatmap on existing axes
            if nargin < 2, ax = gca; end
            obj.env.visualize('traffic', true, 'ax', ax);
        end

        function visualizeWavefront(obj, waveMap, ax)
            % Show wavefront cost map as overlay
            if nargin < 3, ax = gca; end
            hold(ax, 'on');
            wNorm = waveMap ./ max(waveMap(isfinite(waveMap)));
            wNorm(~isfinite(wNorm)) = 0;
            % Cyan contour overlay
            contour(ax, wNorm, 8, 'Color', [0 0.8 0.8], 'LineWidth', 0.8, 'LineStyle', ':');
            hold(ax, 'off');
        end
    end
end