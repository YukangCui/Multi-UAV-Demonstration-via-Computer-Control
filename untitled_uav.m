%% ==== UAV Path Planning with Vertical Oscillating Obstacles (Profiles Fixed) ====
clear; clc; close all;

%% 基本参数
gridSize = [100,100];
numBuildings = 80;
maxBuildingHeight = 40;
minBuildingHeight = 3;
clearance = 6;
maxAllowedHeightToFly = 60; % 提高上限以便显示更明显高度
rng(1);

cols = gridSize(1); rows = gridSize(2);
[X,Y] = meshgrid(1:cols,1:rows);
H = zeros(rows,cols);

%% 生成建筑高度场
for i=1:numBuildings
    cx = randi([10,cols-10]); cy = randi([10,rows-10]);
    bw = randi([6,15]); bh = randi([6,15]);
    h = randi([minBuildingHeight,maxBuildingHeight]);
    shape = randi(3);
    switch shape
        case 1
            mask = X>cx-bw/2 & X<cx+bw/2 & Y>cy-bh/2 & Y<cy+bh/2;
        case 2
            mask = (X-cx).^2 + (Y-cy).^2 < (min(bw,bh)/2)^2;
        otherwise
            mask = abs(X-cx) + abs(Y-cy) < min(bw,bh)/1.5;
    end
    H(mask) = max(H(mask), h);
end

%% 基础 cost map（高于 maxAllowedHeightToFly 的设为不可飞）
costMapBase = 1 + 2*(H/maxBuildingHeight);
costMapBase(H>maxAllowedHeightToFly) = Inf;

%% 起点/终点
start = [5,5];
goal  = [cols-5,rows-5];
if isinf(costMapBase(start(2),start(1))), start=[10,10]; end
if isinf(costMapBase(goal(2),goal(1))),  goal=[cols-10,rows-10]; end

%% 初始 A* 路径
path = astar(costMapBase, start, goal);
if isempty(path), error('初始路径不可达'); end

%% path 对应飞行高度（基于邻域最大建筑 + clearance）
flightZ = zeros(size(path,1),1);
win = 3;
for i=1:length(flightZ)
    x = path(i,1); y = path(i,2);
    x1 = max(1,x-win); x2 = min(cols,x+win);
    y1 = max(1,y-win); y2 = min(rows,y+win);
    flightZ(i) = max(H(y1:y2,x1:x2),[],'all') + clearance;
end
flightZ = movmean(flightZ,5);

%% 动态障碍（水平移动 + 垂直振荡），参数调大以便明显
numDynObs = 8;
obsMinR = 5; obsMaxR = 9;
obsMinSpeed = 0.3; obsMaxSpeed = 1.0;
obsMinH = 12; obsMaxH = 30;
dynObs = struct('pos',[],'r',[],'speed',[],'angle',[],'dir',[],'h_base',[],'A',[],'f',[],'h',[]);
for k=1:numDynObs
    dynObs(k).pos = [randi([15,cols-15]), randi([15,rows-15])];
    dynObs(k).r = randi([obsMinR, obsMaxR]);
    dynObs(k).speed = obsMinSpeed + (obsMaxSpeed-obsMinSpeed)*rand();
    dynObs(k).angle = rand()*2*pi;
    dynObs(k).dir = 1 - 2*(rand>0.5);
    dynObs(k).h_base = obsMinH + (obsMaxH-obsMinH)*rand();
    dynObs(k).A = 8 + 4*rand();    % 振幅大
    dynObs(k).f = 0.03 + 0.02*rand(); % 频率较高
    dynObs(k).h = dynObs(k).h_base; % 初始化当前高度
end

%% 预计算 requiredZ（格点所需飞行高度）用于高度感知阻塞
requiredZ = zeros(rows,cols);
for yy=1:rows
    for xx=1:cols
        x1 = max(1,xx-win); x2 = min(cols,xx+win);
        y1 = max(1,yy-win); y2 = min(rows,yy+win);
        requiredZ(yy,xx) = max(H(y1:y2,x1:x2),[],'all') + clearance;
    end
end

%% 可视化布局：主 3D + top + side(Y-Z) + front(X-Z)
fig = figure('Color','w','Position',[60 60 1600 700]);

% 主视（3D）
axMain = axes('Position',[0.03 0.12 0.55 0.82]);
surf(axMain, 1:cols, 1:rows, H, 'EdgeColor','none','FaceAlpha',0.45); hold(axMain,'on');
hPath3D = plot3(axMain, path(:,1), path(:,2), flightZ, 'k--','LineWidth',1.6);
hStart3D = plot3(axMain, start(1), start(2), flightZ(1), 'go','MarkerFaceColor','g','MarkerSize',8);
hGoal3D  = plot3(axMain, goal(1),  goal(2),  flightZ(end), 'ro','MarkerFaceColor','r','MarkerSize',8);
xlabel(axMain,'X'); ylabel(axMain,'Y'); zlabel(axMain,'Z');
title(axMain,'主视 (3D)'); axis(axMain,'equal'); view(axMain,35,28); camlight(axMain);
maxDisplayZ = max(max(H)) + 20; % 用于设置 profile 的 ylim

% 俯视
axTop = axes('Position',[0.61 0.62 0.36 0.36]);
surf(axTop, 1:cols, 1:rows, H, 'EdgeColor','none','FaceAlpha',0.45); hold(axTop,'on');
hPathTop = plot(axTop, path(:,1), path(:,2), 'k--','LineWidth',1.4);
plot(axTop, start(1), start(2),'go','MarkerFaceColor','g','MarkerSize',8);
plot(axTop, goal(1),  goal(2), 'ro','MarkerFaceColor','r','MarkerSize',8);
axis(axTop,'equal'); view(axTop,2); title(axTop,'俯视 (Top)');

% 侧视 Y-Z：x 轴为 Y（行），y 轴为 Z（高度）
axSide = axes('Position',[0.61 0.34 0.36 0.22]); hold(axSide,'on');
xlabel(axSide,'Y'); ylabel(axSide,'Z'); title(axSide,'侧视 (Y - Z)');
xlim(axSide,[1 rows]); ylim(axSide,[0 maxDisplayZ+10]);
grid(axSide,'on');

% 前视 X-Z：x 轴为 X（列），y 轴为 Z（高度）
axFront = axes('Position',[0.61 0.06 0.36 0.22]); hold(axFront,'on');
xlabel(axFront,'X'); ylabel(axFront,'Z'); title(axFront,'前视 (X - Z)');
xlim(axFront,[1 cols]); ylim(axFront,[0 maxDisplayZ+10]);
grid(axFront,'on');

% 在侧视/前视上绘制路径高度曲线（初始）
hPathSideCurve = plot(axSide, path(:,2), flightZ, 'b-','LineWidth',1.8);   % Y vs Z
hPathFrontCurve= plot(axFront, path(:,1), flightZ, 'b-','LineWidth',1.8);   % X vs Z

% 无人机球体（3D）以及在 top 的投影
[spx,spy,spz] = sphere(24); droneR = 2.0;
drMain = surf(axMain, spx*droneR+path(1,1), spy*droneR+path(1,2), spz*droneR+flightZ(1), 'FaceColor',[0 0.7 1],'EdgeColor','none');
drTop  = surf(axTop,  spx*droneR+path(1,1), spy*droneR+path(1,2), spz*1+1, 'FaceColor',[0 0.7 1],'EdgeColor','none');
% 在侧/前绘制无人机在剖面的投影点（marker）
drSideMarker  = plot(axSide, path(1,2), flightZ(1), 'o','MarkerFaceColor',[0 0.7 1],'MarkerEdgeColor','none','MarkerSize',8);
drFrontMarker = plot(axFront, path(1,1), flightZ(1), 'o','MarkerFaceColor',[0 0.7 1],'MarkerEdgeColor','none','MarkerSize',8);

%% 障碍物可视化句柄：在 3D/top 使用填充圆盘（top 投影），在 side/front 使用竖直线段
theta = linspace(0,2*pi,64);
obsMainHandles = gobjects(numDynObs,1);
obsTopHandles  = gobjects(numDynObs,1);
obsSideLines   = gobjects(numDynObs,1);
obsFrontLines  = gobjects(numDynObs,1);
for k=1:numDynObs
    xc = dynObs(k).pos(1) + dynObs(k).r*cos(theta);
    yc = dynObs(k).pos(2) + dynObs(k).r*sin(theta);
    ztop = dynObs(k).h * ones(size(theta));
    % 3D 顶面（显著红色半透明）
    obsMainHandles(k) = patch(axMain, 'XData', xc, 'YData', yc, 'ZData', ztop, ...
        'FaceColor',[1 0.2 0.2], 'FaceAlpha', 0.65, 'EdgeColor','none');
    % top 投影
    obsTopHandles(k) = patch(axTop, 'XData', xc, 'YData', yc, ...
        'FaceColor',[1 0.2 0.2], 'FaceAlpha', 0.55, 'EdgeColor','none');
    % side 上为竖直线（Y axis 固定）
    obsSideLines(k) = line(axSide, [dynObs(k).pos(2), dynObs(k).pos(2)], [0, dynObs(k).h], ...
        'Color',[1 0 0], 'LineWidth', 3, 'LineStyle','-');
    % front 上为竖直线（X axis 固定）
    obsFrontLines(k) = line(axFront, [dynObs(k).pos(1), dynObs(k).pos(1)], [0, dynObs(k).h], ...
        'Color',[1 0 0], 'LineWidth', 3, 'LineStyle','-');
end

%% GIF 输出设置
gifFile = 'drone_flight_dynamic_obstacles_vertical_stronger_profile.gif';
if exist(gifFile,'file'), delete(gifFile); end
frameCount = 0;

%% 主循环：每帧更新障碍（水平移动 + 垂直振荡）、检测阻塞并在必要时重规划，更新显示
activePath = path; activeFlightZ = flightZ;
currentIdx = 1;
currentPos = path(1,:);

maxFrames = 900;
for t = 1:maxFrames
    % 时间量（控制振荡相位）
    time = t / 20;
    % 更新动态障碍：水平位移 + 垂直振荡
    for k=1:numDynObs
        % 水平移动（小步长）
        vx = dynObs(k).speed * cos(dynObs(k).angle) * dynObs(k).dir;
        vy = dynObs(k).speed * sin(dynObs(k).angle) * dynObs(k).dir;
        dynObs(k).pos = dynObs(k).pos + [vx, vy];
        % 边界反弹
        if dynObs(k).pos(1) < 8 || dynObs(k).pos(1) > cols-8
            dynObs(k).dir = -dynObs(k).dir;
            dynObs(k).angle = dynObs(k).angle + pi*0.2*randn();
        end
        if dynObs(k).pos(2) < 8 || dynObs(k).pos(2) > rows-8
            dynObs(k).dir = -dynObs(k).dir;
            dynObs(k).angle = dynObs(k).angle + pi*0.2*randn();
        end
        % 垂直振荡（显著）
        dynObs(k).h = dynObs(k).h_base + dynObs(k).A * sin(2*pi*dynObs(k).f * time);
        % 保证 h >= 0
        if dynObs(k).h < 0, dynObs(k).h = 0; end
    end

    % 构造当前 costMap（只有当障碍顶高 >= requiredZ 时才阻挡对应格）
    costMapNow = costMapBase;
    [gridX, gridY] = meshgrid(1:cols, 1:rows);
    obsMask = false(rows,cols);
    for k=1:numDynObs
        dx = gridX - dynObs(k).pos(1);
        dy = gridY - dynObs(k).pos(2);
        coverMask = (dx.^2 + dy.^2) <= (dynObs(k).r).^2;
        heightMask = dynObs(k).h >= requiredZ;
        obsMask = obsMask | (coverMask & heightMask);
    end
    costMapNow(obsMask) = Inf;

    % 检查 activePath 是否与当前障碍冲突
    conflict = false;
    for pi = 1:size(activePath,1)
        px = activePath(pi,1); py = activePath(pi,2);
        if obsMask(py, px)
            conflict = true; break;
        end
    end

    % 若冲突，尝试从当前位置重规划
    curGrid = round(currentPos);
    curGrid(1) = min(max(curGrid(1),1),cols);
    curGrid(2) = min(max(curGrid(2),1),rows);
    if isinf(costMapNow(curGrid(2), curGrid(1)))
        % 若当前位置被阻挡，寻找邻域可用格
        neigh = [0 0;1 0;-1 0;0 1;0 -1;1 1;1 -1;-1 1;-1 -1];
        found = false;
        for n = 1:size(neigh,1)
            nx = curGrid(1) + neigh(n,1); ny = curGrid(2) + neigh(n,2);
            if nx>=1 && nx<=cols && ny>=1 && ny<=rows && ~isinf(costMapNow(ny,nx))
                curGrid = [nx, ny]; found = true; break;
            end
        end
        if ~found
            % 被完全包围：等待下一帧（障碍会移动）
            % 不做进一步处理
        end
    end

    if conflict
        newPath = astar(costMapNow, curGrid, goal);
        if ~isempty(newPath)
            activePath = newPath;
            % 重新计算 flightZ
            newFlightZ = zeros(size(activePath,1),1);
            for i=1:length(newFlightZ)
                x = activePath(i,1); y = activePath(i,2);
                x1 = max(1,x-win); x2 = min(cols,x+win);
                y1 = max(1,y-win); y2 = min(rows,y+win);
                newFlightZ(i) = max(H(y1:y2,x1:x2),[],'all') + clearance;
            end
            activeFlightZ = movmean(newFlightZ,5);
            % 把当前位置插入路径起点（若需要）
            if ~all(activePath(1,:)==curGrid)
                activePath = [curGrid; activePath];
                activeFlightZ = [activeFlightZ(1); activeFlightZ];
            end
            currentIdx = 1;
        else
            % 重规划失败，等待下一帧
        end
    end

    % 移动无人机到 activePath 的下一个点（格点跳动）
    if currentIdx < size(activePath,1)
        currentIdx = currentIdx + 1;
        currentPos = activePath(currentIdx,:);
        zNow = activeFlightZ(currentIdx);
    else
        disp('到达目标'); break;
    end

    % 更新无人机 3D 和剖面投影
    set(drMain, 'XData', spx*droneR + currentPos(1), 'YData', spy*droneR + currentPos(2), 'ZData', spz*droneR + zNow);
    set(drTop,  'XData', spx*droneR + currentPos(1), 'YData', spy*droneR + currentPos(2), 'ZData', spz*1 + 1);
    set(drSideMarker,  'XData', currentPos(2), 'YData', zNow);
    set(drFrontMarker, 'XData', currentPos(1), 'YData', zNow);

    % 更新路径曲线在剖面图上的显示
    set(hPath3D, 'XData', activePath(:,1), 'YData', activePath(:,2), 'ZData', activeFlightZ);
    set(hPathTop,'XData', activePath(:,1), 'YData', activePath(:,2));
    set(hPathSideCurve, 'XData', activePath(:,2), 'YData', activeFlightZ);
    set(hPathFrontCurve,'XData', activePath(:,1), 'YData', activeFlightZ);

    % 更新障碍显示（3D 顶面 & top 投影 & 剖面竖线）
    for k=1:numDynObs
        xc = dynObs(k).pos(1) + dynObs(k).r * cos(theta);
        yc = dynObs(k).pos(2) + dynObs(k).r * sin(theta);
        zt = dynObs(k).h * ones(size(theta));
        set(obsMainHandles(k), 'XData', xc, 'YData', yc, 'ZData', zt);
        set(obsTopHandles(k),  'XData', xc, 'YData', yc);
        set(obsSideLines(k),   'XData', [dynObs(k).pos(2), dynObs(k).pos(2)], 'YData', [0, dynObs(k).h]);
        set(obsFrontLines(k),  'XData', [dynObs(k).pos(1), dynObs(k).pos(1)], 'YData', [0, dynObs(k).h]);
    end

    drawnow limitrate nocallbacks;

    % 保存 GIF 帧
    frameCount = frameCount + 1;
    frame = getframe(fig);
    [A,map] = rgb2ind(frame2im(frame),256);
    if frameCount == 1
        imwrite(A,map,gifFile,'gif','LoopCount',Inf,'DelayTime',0.05);
    else
        imwrite(A,map,gifFile,'gif','WriteMode','append','DelayTime',0.05);
    end
end

disp(['动画已保存为: ', fullfile(pwd,gifFile)]);

%% ========== A* 函数 ==========
function path = astar(costMap, start, goal)
[rows,cols] = size(costMap);
g = inf(rows,cols); f = inf(rows,cols);
came = zeros(rows,cols,2); open = false(rows,cols);
g(start(2),start(1)) = 0;
f(start(2),start(1)) = heur(start,goal);
open(start(2),start(1)) = true;
nbr = [1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1];
while any(open(:))
    tmpf = f; tmpf(~open) = inf;
    [~, id] = min(tmpf(:));
    [cy, cx] = ind2sub([rows, cols], id);
    if isequal([cx,cy], goal)
        path = rebuild(came, [cx,cy], start);
        return;
    end
    open(cy,cx) = false;
    for k=1:8
        nx = cx + nbr(k,1); ny = cy + nbr(k,2);
        if nx<1 || nx>cols || ny<1 || ny>rows || isinf(costMap(ny,nx)), continue; end
        tentative = g(cy,cx) + norm(nbr(k,:)) * ((costMap(cy,cx) + costMap(ny,nx))/2);
        if tentative < g(ny,nx)
            came(ny,nx,:) = [cx, cy];
            g(ny,nx) = tentative;
            f(ny,nx) = tentative + heur([nx,ny], goal);
            open(ny,nx) = true;
        end
    end
end
path = [];
end

function h = heur(a,b)
h = hypot(a(1)-b(1), a(2)-b(2));
end

function p = rebuild(came, cur, start)
p = [];
while ~(cur(1)==start(1) && cur(2)==start(2))
    p = [p; cur];
    cur = squeeze(came(cur(2), cur(1), :))';
    if all(cur==0), break; end
end
p = [p; start];
p = flipud(p);
end
