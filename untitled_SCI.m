%% ==== Drone flight visualization: main 3D + small profile views ====
clear; clc; close all;

%% @叫我小睿就好了的参数设置
gridSize = [100,100];
numBuildings = 80;
maxBuildingHeight = 40;
minBuildingHeight = 3;
clearance = 6;
maxAllowedHeightToFly = 45;
rng(1);

%% @叫我小睿就好了模拟建筑高度场生成
cols = gridSize(1); rows = gridSize(2);
H = zeros(rows,cols);
[X,Y]=meshgrid(1:cols,1:rows);

for i=1:numBuildings
    cx=randi([10,cols-10]); cy=randi([10,rows-10]);
    bw=randi([6,15]); bh=randi([6,15]);
    h=randi([minBuildingHeight,maxBuildingHeight]);
    shape=randi(3);
    switch shape
        case 1
            mask=X>cx-bw/2 & X<cx+bw/2 & Y>cy-bh/2 & Y<cy+bh/2;
        case 2
            mask=(X-cx).^2+(Y-cy).^2<(min(bw,bh)/2)^2;
        otherwise
            mask=abs(X-cx)+abs(Y-cy)<min(bw,bh)/1.5;
    end
    H(mask)=max(H(mask),h);
end

%% 成本地图
costMap = 1 + 2*(H/maxBuildingHeight);
costMap(H>maxAllowedHeightToFly)=Inf;

%% 起点终点设置
start=[5,5];
goal=[cols-5,rows-5];
if isinf(costMap(start(2),start(1))), start=[10,10]; end
if isinf(costMap(goal(2),goal(1))), goal=[cols-10,rows-10]; end

%% A* 路径规划
path = astar(costMap,start,goal);
if isempty(path), error('路径不可达'); end

%% 飞行高度设置
flightZ=zeros(size(path,1),1);
for i=1:length(flightZ)
    x=path(i,1); y=path(i,2);
    win=3;
    x1=max(1,x-win);x2=min(cols,x+win);
    y1=max(1,y-win);y2=min(rows,y+win);
    local=max(H(y1:y2,x1:x2),[],'all');
    flightZ(i)=local+clearance;
end
flightZ=movmean(flightZ,5);

%% @叫我小睿就好了布局设计
figure('Color','w','Position',[100 80 1400 600]);

% 主图位置（大）
axMain = axes('Position',[0.05 0.15 0.55 0.75]);
% 小图（右侧三个剖面）
axTop  = axes('Position',[0.68 0.68 0.28 0.25]);   % 俯视
axSide = axes('Position',[0.68 0.38 0.28 0.25]);   % 侧视 (Y-Z)
axFront= axes('Position',[0.68 0.08 0.28 0.25]);   % 前视 (X-Z)

%% @叫我小睿就好了建筑体渲染样式参数设置
cmap = [0.8 0.85 1; 1 0.95 0.8; 0.9 0.9 0.9; 0.85 1 0.9];
baseColor = cmap(randi(size(cmap,1),1),:);

%% ====== 主三维视角的定义 ======
axes(axMain);
surf(1:cols,1:rows,H,'EdgeColor','none','FaceAlpha',0.45,'FaceColor',baseColor);
hold on;
plot3(path(:,1),path(:,2),flightZ,'k--','LineWidth',1.3);
plot3(start(1),start(2),flightZ(1),'go','MarkerFaceColor','g');
plot3(goal(1),goal(2),flightZ(end),'ro','MarkerFaceColor','r');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('主视角（3D 透视）');
axis equal; view(35,25);
lighting gouraud; camlight; material dull;

%% === 剖面1：俯视 ===
axes(axTop);
surf(1:cols,1:rows,H,'EdgeColor','none','FaceAlpha',0.5,'FaceColor',baseColor);
hold on;
plot(path(:,1),path(:,2),'k--','LineWidth',1.1);
plot(start(1),start(2),'go','MarkerFaceColor','g');
plot(goal(1),goal(2),'ro','MarkerFaceColor','r');
axis equal tight; view(2);
title('俯视 (Top view)'); xlabel('X'); ylabel('Y');

%% === 剖面2：侧视 (Y-Z) ===
axes(axSide);
surf(Y,H,'EdgeColor','none','FaceAlpha',0.45,'FaceColor',baseColor);
hold on;
plot3(path(:,2),flightZ,zeros(size(flightZ)),'k--','LineWidth',1.2);
title('侧视 (Y-Z)'); xlabel('Y'); ylabel('Z');
view([0 0]); axis tight;

%% === 剖面3：前视 (X-Z) ===
axes(axFront);
surf(X,H,'EdgeColor','none','FaceAlpha',0.45,'FaceColor',baseColor);
hold on;
plot3(path(:,1),flightZ,zeros(size(flightZ)),'k--','LineWidth',1.2);
title('前视 (X-Z)'); xlabel('X'); ylabel('Z');
view([90 0]); axis tight;

%% === @叫我小睿就好了无人机模型参数设置 ===
[spx,spy,spz]=sphere(24);
r=2.0; % 此处可以调整无人机的大小哦（用圆球表示UAV）
drColor=[0 0.8 1]; % UAV颜色设置
drMain=surf(axMain,spx*r+path(1,1),spy*r+path(1,2),spz*r+flightZ(1), ...
    'FaceColor',drColor,'EdgeColor','none','FaceLighting','gouraud','SpecularStrength',0.9);
drTop =surf(axTop, spx*r+path(1,1),spy*r+path(1,2),spz*r+1, ...
    'FaceColor',drColor,'EdgeColor','none');
drSide=surf(axSide,spx*r+path(1,2),spy*r+1,spz*r+flightZ(1), ...
    'FaceColor',drColor,'EdgeColor','none');
drFront=surf(axFront,spx*r+path(1,1),spy*r+1,spz*r+flightZ(1), ...
    'FaceColor',drColor,'EdgeColor','none');

%% 动画输出效果设置
gifFile='drone_flight_final.gif';
if exist(gifFile,'file'), delete(gifFile); end

for i=1:length(path)
    % 更新无人机位置
    set(drMain,'XData',spx*r+path(i,1),'YData',spy*r+path(i,2),'ZData',spz*r+flightZ(i));
    set(drTop, 'XData',spx*r+path(i,1),'YData',spy*r+path(i,2),'ZData',spz*r+1);
    set(drSide,'XData',spx*r+path(i,2),'YData',spy*r+1,'ZData',spz*r+flightZ(i));
    set(drFront,'XData',spx*r+path(i,1),'YData',spy*r+1,'ZData',spz*r+flightZ(i));
    drawnow limitrate nocallbacks;

    % 将结果保存为GIF帧（从而可以实现动图效果哦）
    frame=getframe(gcf);
    [A,map]=rgb2ind(frame2im(frame),256);
    if i==1
        imwrite(A,map,gifFile,'gif','LoopCount',Inf,'DelayTime',0.05);
    else
        imwrite(A,map,gifFile,'gif','WriteMode','append','DelayTime',0.05);
    end
end

disp(['UAV飞行动画已保存为 ',fullfile(pwd,gifFile)]);

%% ======= A* 函数 =======@关键函数定义说明
function path=astar(costMap,start,goal)
[rows,cols]=size(costMap);
g=inf(rows,cols); f=inf(rows,cols);
came=zeros(rows,cols,2); open=false(rows,cols);
g(start(2),start(1))=0; f(start(2),start(1))=heur(start,goal); open(start(2),start(1))=true;
nbr=[1 0;-1 0;0 1;0 -1;1 1;1 -1;-1 1;-1 -1];
while any(open(:))
    [~,id]=min(f(:)+(~open(:))*1e9);
    [cy,cx]=ind2sub([rows,cols],id);
    if [cx,cy]==goal, path=rebuild(came,[cx,cy],start);return; end
    open(cy,cx)=false;
    for k=1:8
        nx=cx+nbr(k,1); ny=cy+nbr(k,2);
        if nx<1||nx>cols||ny<1||ny>rows||isinf(costMap(ny,nx)),continue;end
        tmp=g(cy,cx)+norm(nbr(k,:))*((costMap(cy,cx)+costMap(ny,nx))/2);
        if tmp<g(ny,nx)
            came(ny,nx,:)=[cx,cy];
            g(ny,nx)=tmp;
            f(ny,nx)=tmp+heur([nx,ny],goal);
            open(ny,nx)=true;
        end
    end
end
path=[];
end

function h=heur(a,b)
h=hypot(a(1)-b(1),a(2)-b(2));
end

function p=rebuild(came,cur,start)
p=[];
while ~(cur(1)==start(1)&&cur(2)==start(2))
    p=[p;cur];
    cur=squeeze(came(cur(2),cur(1),:))';
    if all(cur==0),break;end
end
p=[p;start];
p=flipud(p);
end
