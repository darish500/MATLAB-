classdef Environment < handle
    % ENVIRONMENT - Grid world for MAPF-RO simulation
    %
    % Represents the grid environment described in the paper.
    % Cell types:
    %   0 = FREE space
    %   1 = STATIC obstacle (walls, immovable)
    %   2 = PIT (removable obstacle - filled by sandbags)
    %   3 = SANDBAG (movable obstacle - can be pushed into pits)
    %
    % Usage:
    %   env = Environment(20, 20);          % empty 20x20 grid
    %   env = Environment(20, 20, 'random', nRobots, pitRatio); % random
    %   env.visualize();                    % see the grid
    %
    % Paper reference: Section III - Problem Definition

    %% ── Constants (cell type codes) ────────────────────────────────────
    properties (Constant)
        FREE    = 0
        STATIC  = 1
        PIT     = 2
        SANDBAG = 3
    end

    %% ── Core grid data ──────────────────────────────────────────────────
    properties
        rows        (1,1) double  % grid height
        cols        (1,1) double  % grid width
        grid        (:,:) double  % main grid matrix  [rows x cols]

        % Robot start/goal positions  [n x 2] each row = [row, col]
        robotStarts (:,2) double
        robotGoals  (:,2) double
        nRobots     (1,1) double

        % Pit-to-sandbag ratio string used during generation
        pitSandbagRatio (1,2) double = [1 1]   % e.g. [1 2] means 1:2

        % Traffic index matrix (updated by planner, initialised to zero)
        trafficIndex (:,:) double
    end

    %% ── Derived / cached ────────────────────────────────────────────────
    properties (Dependent)
        pitPositions     % [m x 2] positions of all pits
        sandbagPositions % [m x 2] positions of all sandbags
        freePositions    % [m x 2] all free cells
    end

    %% ════════════════════════════════════════════════════════════════════
    %  CONSTRUCTOR
    %% ════════════════════════════════════════════════════════════════════
    methods
        function obj = Environment(rows, cols, mode, nRobots, ratio)
            % Environment(rows, cols)
            % Environment(rows, cols, 'empty')
            % Environment(rows, cols, 'random', nRobots, [pit sandbag])
            %
            % ratio example: [1 2] = 1 pit for every 2 sandbags

            arguments
                rows    (1,1) double {mustBePositive, mustBeInteger}
                cols    (1,1) double {mustBePositive, mustBeInteger}
                mode    (1,:) char   = 'empty'
                nRobots (1,1) double = 3
                ratio   (1,2) double = [1 1]
            end

            obj.rows         = rows;
            obj.cols         = cols;
            obj.grid         = zeros(rows, cols);   % all FREE
            obj.trafficIndex = zeros(rows, cols);
            obj.nRobots      = nRobots;
            obj.pitSandbagRatio = ratio;

            switch lower(mode)
                case 'random'
                    obj.generateRandom(nRobots, ratio);
                case 'empty'
                    % just initialise robot arrays as empty
                    obj.robotStarts = zeros(0,2);
                    obj.robotGoals  = zeros(0,2);
                otherwise
                    error('Environment: unknown mode "%s". Use "empty" or "random".', mode);
            end
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  DEPENDENT PROPERTIES
    %% ════════════════════════════════════════════════════════════════════
    methods
        function pos = get.pitPositions(obj)
            [r, c] = find(obj.grid == obj.PIT);
            pos = [r, c];
        end

        function pos = get.sandbagPositions(obj)
            [r, c] = find(obj.grid == obj.SANDBAG);
            pos = [r, c];
        end

        function pos = get.freePositions(obj)
            [r, c] = find(obj.grid == obj.FREE);
            pos = [r, c];
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  PUBLIC METHODS
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = public)

        % ── Cell query helpers ───────────────────────────────────────────
        function tf = isFree(obj, r, c)
            tf = obj.inBounds(r,c) && obj.grid(r,c) == obj.FREE;
        end

        function tf = isPit(obj, r, c)
            tf = obj.inBounds(r,c) && obj.grid(r,c) == obj.PIT;
        end

        function tf = isSandbag(obj, r, c)
            tf = obj.inBounds(r,c) && obj.grid(r,c) == obj.SANDBAG;
        end

        function tf = isStatic(obj, r, c)
            tf = obj.inBounds(r,c) && obj.grid(r,c) == obj.STATIC;
        end

        function tf = isTraversable(obj, r, c)
            % A cell a robot CAN occupy (free or pit - with penalty)
            tf = obj.inBounds(r,c) && obj.grid(r,c) ~= obj.STATIC;
        end

        function tf = inBounds(obj, r, c)
            tf = r >= 1 && r <= obj.rows && c >= 1 && c <= obj.cols;
        end

        % ── Environment modification ─────────────────────────────────────
        function fillPit(obj, r, c)
            % Fill a pit with a sandbag → becomes FREE
            if obj.isPit(r, c)
                obj.grid(r, c) = obj.FREE;
            else
                warning('Environment.fillPit: cell (%d,%d) is not a pit.', r, c);
            end
        end

        function removeSandbag(obj, r, c)
            % Remove a sandbag from grid
            if obj.isSandbag(r, c)
                obj.grid(r, c) = obj.FREE;
            else
                warning('Environment.removeSandbag: cell (%d,%d) is not a sandbag.', r, c);
            end
        end

        function moveSandbag(obj, fromR, fromC, toR, toC)
            % Move sandbag from one cell to another
            if obj.isSandbag(fromR, fromC) && obj.isFree(toR, toC)
                obj.grid(fromR, fromC) = obj.FREE;
                obj.grid(toR,   toC)   = obj.SANDBAG;
            else
                warning('Environment.moveSandbag: invalid move (%d,%d)->(%d,%d).', ...
                    fromR, fromC, toR, toC);
            end
        end

        function setCellType(obj, r, c, type)
            % Directly set a cell type (use Environment.FREE/STATIC/etc)
            if obj.inBounds(r, c)
                obj.grid(r, c) = type;
            end
        end

        % ── Traffic index ────────────────────────────────────────────────
        function resetTraffic(obj)
            obj.trafficIndex = zeros(obj.rows, obj.cols);
        end

        function incrementTraffic(obj, path)
            % path: [n x 2] list of [row col] waypoints
            for i = 1:size(path, 1)
                r = path(i,1); c = path(i,2);
                if obj.inBounds(r,c)
                    obj.trafficIndex(r,c) = obj.trafficIndex(r,c) + 1;
                end
            end
        end

        % ── Neighbours (4-connected, paper assumption) ───────────────────
        function nbrs = getNeighbours(obj, r, c)
            % Returns [k x 2] of valid traversable neighbours
            candidates = [r-1,c; r+1,c; r,c-1; r,c+1];
            valid = candidates(:,1)>=1 & candidates(:,1)<=obj.rows & ...
                    candidates(:,2)>=1 & candidates(:,2)<=obj.cols;
            candidates = candidates(valid, :);
            % Remove static obstacles
            keep = arrayfun(@(i) obj.grid(candidates(i,1), candidates(i,2)) ~= obj.STATIC, ...
                            1:size(candidates,1));
            nbrs = candidates(keep, :);
        end

        % ── Snapshot (for undo / replanning) ────────────────────────────
        function snap = snapshot(obj)
            % Returns a struct copy of critical state
            snap.grid         = obj.grid;
            snap.trafficIndex = obj.trafficIndex;
        end

        function restoreSnapshot(obj, snap)
            obj.grid         = snap.grid;
            obj.trafficIndex = snap.trafficIndex;
        end

        % ── VISUALIZE ────────────────────────────────────────────────────
        function h = visualize(obj, varargin)
            % visualize()               → plain grid
            % visualize('traffic', true) → overlay traffic heatmap
            % visualize('ax', axHandle)  → draw into existing axes
            %
            % Returns handle to the axes.

            p = inputParser;
            addParameter(p, 'traffic', false, @islogical);
            addParameter(p, 'ax',      [],    @(x) isa(x,'matlab.graphics.axis.Axes'));
            addParameter(p, 'title',   '',    @ischar);
            parse(p, varargin{:});

            if isempty(p.Results.ax)
                figure('Name','MAPF-RO Environment', ...
                       'Color',[0.12 0.12 0.15], ...
                       'NumberTitle','off');
                ax = axes('Color',[0.15 0.15 0.18]);
            else
                ax = p.Results.ax;
            end

            % ── Build RGB image ──────────────────────────────────────────
            %  Colour scheme (dark-theme, Nav2-inspired):
            %   FREE    → dark grey   [0.20 0.20 0.22]
            %   STATIC  → near-black  [0.08 0.08 0.10]
            %   PIT     → slate blue  [0.25 0.35 0.55]
            %   SANDBAG → warm brown  [0.65 0.40 0.15]

            colorFree    = [0.20 0.20 0.22];
            colorStatic  = [0.08 0.08 0.10];
            colorPit     = [0.25 0.45 0.75];
            colorSandbag = [0.75 0.50 0.15];

            img = zeros(obj.rows, obj.cols, 3);
            for ch = 1:3
                layer = zeros(obj.rows, obj.cols);
                layer(obj.grid == obj.FREE)    = colorFree(ch);
                layer(obj.grid == obj.STATIC)  = colorStatic(ch);
                layer(obj.grid == obj.PIT)     = colorPit(ch);
                layer(obj.grid == obj.SANDBAG) = colorSandbag(ch);
                img(:,:,ch) = layer;
            end

            % ── Traffic overlay ──────────────────────────────────────────
            if p.Results.traffic && any(obj.trafficIndex(:) > 0)
                tNorm = obj.trafficIndex ./ max(obj.trafficIndex(:));
                % Blend red channel with traffic intensity
                img(:,:,1) = img(:,:,1) + tNorm * 0.4;
                img(:,:,2) = img(:,:,2) - tNorm * 0.1;
                img = min(img, 1);
            end

            % ── Draw grid image ──────────────────────────────────────────
            imagesc(ax, img);
            axis(ax, 'equal', 'tight');
            ax.XColor = [0.6 0.6 0.6];
            ax.YColor = [0.6 0.6 0.6];
            ax.GridColor = [0.3 0.3 0.3];
            ax.GridAlpha = 0.4;
            ax.XGrid = 'on';
            ax.YGrid = 'on';
            ax.XTick = 0.5:obj.cols+0.5;
            ax.YTick = 0.5:obj.rows+0.5;
            ax.XTickLabel = {};
            ax.YTickLabel = {};
            ax.TickLength = [0 0];

            % ── Draw grid lines ──────────────────────────────────────────
            hold(ax, 'on');

            % ── Robot starts (blue circles) ──────────────────────────────
            if ~isempty(obj.robotStarts)
                scatter(ax, obj.robotStarts(:,2), obj.robotStarts(:,1), ...
                    120, 'o', ...
                    'MarkerFaceColor', [0.20 0.60 1.00], ...
                    'MarkerEdgeColor', [1 1 1], ...
                    'LineWidth', 1.5, ...
                    'DisplayName', 'Robot Start');

                % Label robot numbers
                for i = 1:size(obj.robotStarts,1)
                    text(ax, obj.robotStarts(i,2), obj.robotStarts(i,1), ...
                        sprintf('R%d',i), ...
                        'Color','white','FontSize',7,'FontWeight','bold',...
                        'HorizontalAlignment','center','VerticalAlignment','middle');
                end
            end

            % ── Robot goals (green diamonds) ─────────────────────────────
            if ~isempty(obj.robotGoals)
                scatter(ax, obj.robotGoals(:,2), obj.robotGoals(:,1), ...
                    120, 'd', ...
                    'MarkerFaceColor', [0.15 0.85 0.40], ...
                    'MarkerEdgeColor', [1 1 1], ...
                    'LineWidth', 1.5, ...
                    'DisplayName', 'Robot Goal');

                for i = 1:size(obj.robotGoals,1)
                    text(ax, obj.robotGoals(i,2), obj.robotGoals(i,1), ...
                        sprintf('G%d',i), ...
                        'Color','white','FontSize',7,'FontWeight','bold',...
                        'HorizontalAlignment','center','VerticalAlignment','middle');
                end
            end

            % ── Legend ───────────────────────────────────────────────────
            % Manual colour patches for legend
            pFree    = patch(ax, NaN,NaN, colorFree,    'DisplayName','Free');
            pStatic  = patch(ax, NaN,NaN, colorStatic,  'DisplayName','Static Obstacle');
            pPit     = patch(ax, NaN,NaN, colorPit,     'DisplayName','Pit (Removable)');
            pSandbag = patch(ax, NaN,NaN, colorSandbag, 'DisplayName','Sandbag (Movable)');

            lg = legend(ax, [pFree, pStatic, pPit, pSandbag], ...
                'Location','southoutside', 'Orientation','horizontal', ...
                'TextColor',[0.85 0.85 0.85], 'Color',[0.15 0.15 0.18], ...
                'EdgeColor',[0.3 0.3 0.3], 'FontSize', 8);

            % ── Title ────────────────────────────────────────────────────
            titleStr = p.Results.title;
            if isempty(titleStr)
                titleStr = sprintf('MAPF-RO Environment  [%d × %d]  |  %d Robots  |  %d Pits  |  %d Sandbags', ...
                    obj.rows, obj.cols, obj.nRobots, ...
                    size(obj.pitPositions,1), size(obj.sandbagPositions,1));
            end
            title(ax, titleStr, 'Color',[0.90 0.90 0.90], 'FontSize',11, 'FontWeight','bold');

            hold(ax, 'off');
            h = ax;
        end

        % ── Summary printout ─────────────────────────────────────────────
        function info(obj)
            fprintf('\n══════════════════════════════════════════\n');
            fprintf('  MAPF-RO Environment  [%d × %d]\n', obj.rows, obj.cols);
            fprintf('══════════════════════════════════════════\n');
            fprintf('  Robots   : %d\n', obj.nRobots);
            fprintf('  Free     : %d cells\n', sum(obj.grid(:)==obj.FREE));
            fprintf('  Static   : %d cells\n', sum(obj.grid(:)==obj.STATIC));
            fprintf('  Pits     : %d cells\n', sum(obj.grid(:)==obj.PIT));
            fprintf('  Sandbags : %d cells\n', sum(obj.grid(:)==obj.SANDBAG));
            fprintf('  Ratio    : %d pits : %d sandbags\n', ...
                obj.pitSandbagRatio(1), obj.pitSandbagRatio(2));
            fprintf('══════════════════════════════════════════\n\n');
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  PRIVATE METHODS
    %% ════════════════════════════════════════════════════════════════════
    methods (Access = private)

        function generateRandom(obj, nRobots, ratio)
            % Generates a random environment matching paper's setup.
            % Key insight: place robots FIRST, then scatter pits/sandbags
            % along Manhattan corridors so they appear on natural paths.

            totalCells = obj.rows * obj.cols;

            % ── Static obstacles (~15% clustered walls) ──────────────────
            nStatic = round(totalCells * 0.15);
            obj.placeClusteredStatics(nStatic);

            % ── Place robots first in free cells ─────────────────────────
            obj.nRobots = nRobots;
            [starts, goals] = obj.placeRobots(nRobots);
            obj.robotStarts = starts;
            obj.robotGoals  = goals;

            % ── Compute Manhattan corridor cells between each start→goal ──
            corridorCells = obj.getManhattanCorridors(starts, goals);

            % ── Decide pit and sandbag counts ─────────────────────────────
            freeCells  = sum(obj.grid(:) == obj.FREE);
            nObstacles = round(freeCells * 0.10);
            nPits     = max(2, round(nObstacles * ratio(1) / sum(ratio)));
            nSandbags = max(2, round(nObstacles * ratio(2) / sum(ratio)));

            % Place ~50% of pits/sandbags ON corridors (guaranteed on paths)
            nPitsCorridor = round(nPits * 0.5);
            nSBCorridor   = round(nSandbags * 0.5);

            obj.placeOnCorridor(obj.PIT,     corridorCells, nPitsCorridor);
            obj.placeOnCorridor(obj.SANDBAG, corridorCells, nSBCorridor);

            % Remaining placed randomly elsewhere
            obj.placeRandom(obj.PIT,     nPits - nPitsCorridor);
            obj.placeRandom(obj.SANDBAG, nSandbags - nSBCorridor);
        end

        function corridorCells = getManhattanCorridors(obj, starts, goals)
            % Returns cells lying on Manhattan L-shaped paths start->goal
            cellSet = false(obj.rows, obj.cols);
            for i = 1:size(starts,1)
                r1 = starts(i,1); c1 = starts(i,2);
                r2 = goals(i,1);  c2 = goals(i,2);
                % Horizontal leg at r1
                cMin = min(c1,c2); cMax = max(c1,c2);
                for c = cMin:cMax
                    if obj.inBounds(r1,c) && obj.grid(r1,c)==obj.FREE
                        cellSet(r1,c) = true;
                    end
                end
                % Vertical leg at c2
                rMin = min(r1,r2); rMax = max(r1,r2);
                for r = rMin:rMax
                    if obj.inBounds(r,c2) && obj.grid(r,c2)==obj.FREE
                        cellSet(r,c2) = true;
                    end
                end
            end
            % Remove start/goal cells themselves
            for i = 1:size(starts,1)
                if obj.inBounds(starts(i,1),starts(i,2))
                    cellSet(starts(i,1),starts(i,2)) = false;
                end
                if obj.inBounds(goals(i,1),goals(i,2))
                    cellSet(goals(i,1),goals(i,2)) = false;
                end
            end
            [r,c] = find(cellSet);
            corridorCells = [r, c];
        end

        function placeOnCorridor(obj, cellType, corridorCells, n)
            % Place n cells of cellType on free corridor cells
            if isempty(corridorCells) || n <= 0, return; end
            mask = arrayfun(@(i) ...
                obj.grid(corridorCells(i,1),corridorCells(i,2))==obj.FREE, ...
                1:size(corridorCells,1));
            free = corridorCells(mask, :);
            if isempty(free), return; end
            n = min(n, size(free,1));
            idx = randperm(size(free,1), n);
            for i = 1:n
                obj.grid(free(idx(i),1), free(idx(i),2)) = cellType;
            end
        end

        function placeClusteredStatics(obj, n)
            % Places static obstacles as horizontal/vertical wall segments
            % to mimic construction site corridors (like paper's Fig 3)
            placed = 0;
            attempts = 0;
            maxAttempts = n * 20;

            while placed < n && attempts < maxAttempts
                attempts = attempts + 1;

                % Random wall segment
                r = randi(obj.rows);
                c = randi(obj.cols);
                len = randi([2, max(3, round(obj.cols/5))]);
                isHoriz = rand > 0.5;

                for k = 0:len-1
                    if isHoriz
                        cc = c + k;
                        rr = r;
                    else
                        rr = r + k;
                        cc = c;
                    end
                    if obj.inBounds(rr, cc) && obj.grid(rr,cc) == obj.FREE
                        obj.grid(rr, cc) = obj.STATIC;
                        placed = placed + 1;
                        if placed >= n, break; end
                    end
                end
            end
        end

        function placeRandom(obj, cellType, n)
            % Randomly place n cells of a given type in free cells
            free = obj.freePositions;
            if isempty(free)
                warning('Environment: no free cells to place type %d', cellType);
                return;
            end
            n = min(n, size(free,1));
            idx = randperm(size(free,1), n);
            for i = 1:n
                obj.grid(free(idx(i),1), free(idx(i),2)) = cellType;
            end
        end

        function [starts, goals] = placeRobots(obj, n)
            % Place robot starts and goals in free cells, well separated
            free = obj.freePositions;
            if size(free,1) < 2*n
                error('Environment: not enough free cells for %d robots (need %d, have %d).', ...
                    n, 2*n, size(free,1));
            end

            % Shuffle and pick first 2n free cells
            idx = randperm(size(free,1), 2*n);
            starts = free(idx(1:n),     :);
            goals  = free(idx(n+1:2*n), :);
        end
    end

    %% ════════════════════════════════════════════════════════════════════
    %  STATIC FACTORY METHODS  (convenient constructors)
    %% ════════════════════════════════════════════════════════════════════
    methods (Static)
        function env = small(nRobots, ratio)
            % 20×20 environment (paper's small size)
            arguments
                nRobots (1,1) double = 3
                ratio   (1,2) double = [1 1]
            end
            env = Environment(20, 20, 'random', nRobots, ratio);
        end

        function env = medium(nRobots, ratio)
            % 30×30 environment (paper's medium size)
            arguments
                nRobots (1,1) double = 5
                ratio   (1,2) double = [1 1]
            end
            env = Environment(30, 30, 'random', nRobots, ratio);
        end

        function env = large(nRobots, ratio)
            % 40×40 environment (paper's large size)
            arguments
                nRobots (1,1) double = 10
                ratio   (1,2) double = [1 1]
            end
            env = Environment(40, 40, 'random', nRobots, ratio);
        end
    end
end