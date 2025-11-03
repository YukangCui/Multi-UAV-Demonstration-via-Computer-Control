function uav_anim_single(demo_id, OS4_physical_in, initVal_in, varargin)
% UAV 单机动画（服务第1–4章），保持你原来的无人机本体外观（13个surface）
% 特性：
%  1) 姿态由动力学映射：(a + g*e3) → R → (phi,theta,psi)，大半径/高速时不失真
%  2) 自动限速以满足最大倾角 MaxTiltDeg（可调）
%  3) 轨迹清晰：双层描边(白底+黑/蓝)、稀疏标记点，置顶显示
%  4) 视野自适应：AutoFit / FitMargin / SquareXY，或用 XLim/YLim/ZLim 显式指定
%  5) 统一接口：'helix' | 'ch1_model' | 'ch2_pid' | 'ch3_kf' | 'ch4_lqr'
%  6) 轨迹半径控制：'HelixR',R  | 'CircleR',R  | 'PathScale',s
% 用法（例）：
%  uav_anim_single('helix', [], [], 'HelixR', 20, 'MaxTiltDeg', 25, ...
%                  'AutoFit', true, 'FitMargin', 0.2, 'AxisCenter', [0 0]);

if nargin < 1 || isempty(demo_id), demo_id = 'helix'; end
g = 9.81;

%% ========= 依赖初始化（从入参/ init / base / 兜底） =========
OS4_physical = []; initVal = [];
if nargin >= 2 && ~isempty(OS4_physical_in), OS4_physical = OS4_physical_in; end
if nargin >= 3 && ~isempty(initVal_in),      initVal      = initVal_in;      end

if isempty(OS4_physical) || isempty(initVal)
    if exist('init','file')==2
        try
            [OS4_physical_tmp, initVal_tmp] = init();    % 若 init 是函数
            if isempty(OS4_physical), OS4_physical = OS4_physical_tmp; end
            if isempty(initVal),      initVal      = initVal_tmp;      end
        catch
            try init; end                                     % 若 init 是脚本
            if isempty(OS4_physical)
                try OS4_physical = evalin('base','OS4_physical'); end
            end
            if isempty(initVal)
                try initVal = evalin('base','initVal'); end
            end
        end
    end
end
% 兜底尺寸（与原模型兼容）
if isempty(OS4_physical)
    OS4_physical.size.fuselage.x = 0.24;
    OS4_physical.size.fuselage.y = 0.06;
    OS4_physical.size.fuselage.z = 0.04;
    OS4_physical.size.arm.diameter   = 0.012;
    OS4_physical.size.motor.diameter = 0.035;
    OS4_physical.size.motor.height   = 0.02;
    L = 0.18;
    OS4_physical.rotor.pos = [ L,  0, -L,  0;
                               0,  L,  0, -L;
                               0,  0,  0,  0 ];
    OS4_physical.prop.rad = 0.09;
end
if isempty(initVal)
    rxy = sqrt(sum(OS4_physical.rotor.pos(1:2,:).^2,1));
    initVal.rr  = max(rxy) + OS4_physical.prop.rad;
    initVal.x00 = 0;  initVal.y00 = 0;
end

%% ========= 可选参数解析 =========
opts.PathScale   = 1.0;     % 全局平面缩放
opts.HelixR      = [];      % 螺旋半径（m）
opts.CircleR     = [];      % 圆轨迹半径（m）
opts.MaxTiltDeg  = 25;      % 最大倾角（度）
opts.AutoFit     = true;    % 是否自动适配视野
opts.FitMargin   = 0.15;    % 视野边距比例
opts.SquareXY    = true;    % XY 对称等尺度
opts.XLim = []; opts.YLim = []; opts.ZLim = [];   % 显式轴限
opts.AxisCenter  = [];      % 指定中心 [cx cy]
for k = 1:2:numel(varargin)
    if k+1<=numel(varargin) && ischar(varargin{k})
        key = lower(varargin{k});
        val = varargin{k+1};
        switch key
            case 'pathscale',   opts.PathScale  = val;
            case 'helixr',      opts.HelixR     = val;
            case 'circler',     opts.CircleR    = val;
            case 'maxtiltdeg',  opts.MaxTiltDeg = val;
            case 'autofit',     opts.AutoFit    = logical(val);
            case 'fitmargin',   opts.FitMargin  = val;
            case 'squarexy',    opts.SquareXY   = logical(val);
            case 'xlim',        opts.XLim       = val;
            case 'ylim',        opts.YLim       = val;
            case 'zlim',        opts.ZLim       = val;
            case 'axiscenter',  opts.AxisCenter = val;
        end
    end
end
theta_max = deg2rad(opts.MaxTiltDeg);

%% ========= 生成示例轨迹（占位：1–4章可替换为你的仿真输出） =========
Ts_disp = 0.02;
switch lower(demo_id)
    case 'helix'
        R0 = 0.6; if ~isempty(opts.HelixR), R0 = opts.HelixR; end
        omega0 = 0.35*2*pi;
        omega  = min(omega0, sqrt(g*tan(theta_max)/max(R0,1e-6)));         % 限速满足最大倾角
        param = struct('R',R0,'omega',omega,'vz',0.12,'Tend',20,'z0',0.2,'theta_max',theta_max);
        traj  = demo_helix_dyn(param, Ts_disp, g);

    case 'ch1_model'
        param = struct('Tend',12,'z_step',0.5);
        traj  = demo_ch1_placeholder(param, Ts_disp);

    case 'ch2_pid'
        R0 = 0.8; if ~isempty(opts.CircleR), R0 = opts.CircleR; end
        om0 = 0.25*2*pi;
        om  = min(om0, sqrt(g*tan(theta_max)/max(R0,1e-6)));
        param = struct('Tend',18,'z_ref',1.0,'circle_R',R0,'circle_omega',om,'theta_max',theta_max);
        traj  = demo_ch2_placeholder_dyn(param, Ts_disp, g);

    case 'ch3_kf'
        param = struct('R',0.5,'omega',0.30*2*pi,'vz',0.08,'Tend',16,'z0',0.3, ...
                       'na',0.02,'ng',deg2rad(0.5),'theta_max',theta_max);
        param.omega = min(param.omega, sqrt(g*tan(theta_max)/max(param.R,1e-6)));
        traj  = demo_ch3_kf_placeholder_dyn(param, Ts_disp, g);

    case 'ch4_lqr'
        param = struct('Tend',12,'xg',0.0,'yg',0.0,'z0',1.5);
        traj  = demo_ch4_lqr_placeholder(param, Ts_disp, g);

    otherwise
        error('未知 demo_id: %s', demo_id);
end

%% ========= 轨迹平面缩放（以中心为基准放大，并重算姿态） =========
traj = applyPathScale_dyn(traj, opts, initVal, g);

%% ========= 图窗与三视图 =========
close(findobj('type','figure','name','UAV Single Animation'));
fig = figure('name','UAV Single Animation','Position',[100, 70, 1080, 780], 'Renderer','opengl');

ax_att = subplot(2,2,1); title(ax_att,'Attitude');  baseAxes(ax_att);
ax_3d  = subplot(2,2,2); title(ax_3d,'3D View');    xlabel(ax_3d,'x'); ylabel(ax_3d,'y'); zlabel(ax_3d,'z');
grid(ax_3d,'on'); view(ax_3d,[-20 30]); set(ax_3d,'SortMethod','childorder');
ax_top = subplot(2,2,[3,4]); title(ax_top,'Top View'); xlabel(ax_top,'x'); ylabel(ax_top,'y');
grid(ax_top,'on'); set(ax_top,'SortMethod','childorder');

% ---- 视野设置：优先显式轴限 → AutoFit → 旧 initVal ----
if ~isempty(opts.XLim) && ~isempty(opts.YLim)
    ax3_x = opts.XLim; ax3_y = opts.YLim;
    if ~isempty(opts.ZLim), ax3_z = opts.ZLim; else, ax3_z = autoZ(traj, 0.1); end
else
    if opts.AutoFit
        [ax3_x, ax3_y, ax3_z] = autoAxesFromTraj(traj, opts);
    else
        if ~isempty(initVal)
            ax3_x = [initVal.x00-2*initVal.rr, 3*initVal.rr];
            ax3_y = [-3*initVal.rr, initVal.y00+2*initVal.rr];
        else
            [ax3_x, ax3_y] = autoXY(traj, 0.15, true, []);
        end
        ax3_z = autoZ(traj, 0.1);
    end
end
set(ax_3d,'XLim',ax3_x,'YLim',ax3_y,'ZLim',ax3_z,'DataAspectRatio',[1 1 1], ...
          'NextPlot','add','XLimMode','manual','YLimMode','manual','ZLimMode','manual');
set(ax_top,'XLim',ax3_x,'YLim',ax3_y,'DataAspectRatio',[1 1 1], ...
           'NextPlot','add','XLimMode','manual','YLimMode','manual');
set(ax_att,'XLim',[-0.4 0.4],'YLim',[-0.4 0.4],'ZLim',[-0.4 0.4]);

%% ========= 构建无人机本体（完全沿用你原样式） =========
combined_att = build_os4_geometry_like_yours(ax_att, OS4_physical);
combined_3d  = build_os4_geometry_like_yours(ax_3d,  OS4_physical);
combined_top = build_os4_geometry_like_yours(ax_top, OS4_physical);

TF_att = hgtransform('Parent',ax_att); set(combined_att,'Parent',TF_att);
TF_3d  = hgtransform('Parent',ax_3d ); set(combined_3d, 'Parent',TF_3d );
TF_top = hgtransform('Parent',ax_top); set(combined_top,'Parent',TF_top);

% 轨迹线：双层描边 + 稀疏标记
h3_bg   = line('Parent',ax_3d, 'XData',NaN,'YData',NaN,'ZData',NaN,'Color',[1 1 1],'LineWidth',5,'Clipping','on','HandleVisibility','off');
h3_true = line('Parent',ax_3d, 'XData',NaN,'YData',NaN,'ZData',NaN,'Color','k',    'LineWidth',2,'Clipping','on');
h3_mk   = line('Parent',ax_3d, 'XData',NaN,'YData',NaN,'ZData',NaN,'Color',[0 0 0],'LineStyle','none','Marker','o','MarkerSize',3,'Clipping','on','HandleVisibility','off');
h2_bg   = line('Parent',ax_top,'XData',NaN,'YData',NaN,'Color',[1 1 1],'LineWidth',5,'Clipping','on','HandleVisibility','off');
h2_true = line('Parent',ax_top,'XData',NaN,'YData',NaN,'Color','k',    'LineWidth',2,'Clipping','on');
h2_mk   = line('Parent',ax_top,'XData',NaN,'YData',NaN,'Color',[0 0 0],'LineStyle','none','Marker','o','MarkerSize',3,'Clipping','on','HandleVisibility','off');

% ch3 估计轨迹（蓝）
if isfield(traj,'est') && ~isempty(traj.est)
    h3_est = line('Parent',ax_3d,'XData',NaN,'YData',NaN,'ZData',NaN,'Color',[0 0.45 0.9],'LineWidth',1.6,'Clipping','on');
    h2_est = line('Parent',ax_top,'XData',NaN,'YData',NaN,'Color',[0 0.45 0.9],'LineWidth',1.6,'Clipping','on');
    uistack([h3_bg h3_true h3_mk h3_est],'top');
    uistack([h2_bg h2_true h2_mk h2_est],'top');
else
    uistack([h3_bg h3_true h3_mk],'top');
    uistack([h2_bg h2_true h2_mk],'top');
end

txt_time = text(0,17,'','Parent',ax_att,'Units','characters','VerticalAlignment','top','HorizontalAlignment','left','FontSize',12);

%% ========= 播放 =========
ax3_view = -20; mkStep = max(5, round(0.10/ Ts_disp));
for i = 2:numel(traj.t)
    if floor(traj.t(i)/Ts_disp) == floor(traj.t(i-1)/Ts_disp), continue; end

    eul = getEuler(traj, i);
    [th, rx, ry, rz] = Eular2axisangle(eul(1), eul(2), eul(3));
    p  = traj.p(i,:);

    TL = makehgtform('translate', p);
    RT = makehgtform('axisrotate', [rx ry rz], th);

    set(TF_att,'Matrix', RT);
    set(TF_3d, 'Matrix', TL*RT);
    set(TF_top,'Matrix', makehgtform('translate',[p(1) p(2) 0])*RT);

    % 轨迹整段赋值
    set(h3_bg,  'XData',traj.p(1:i,1),'YData',traj.p(1:i,2),'ZData',traj.p(1:i,3));
    set(h3_true,'XData',traj.p(1:i,1),'YData',traj.p(1:i,2),'ZData',traj.p(1:i,3));
    set(h2_bg,  'XData',traj.p(1:i,1),'YData',traj.p(1:i,2));
    set(h2_true,'XData',traj.p(1:i,1),'YData',traj.p(1:i,2));

    % 稀疏标记
    idx = 1:mkStep:i;
    set(h3_mk,'XData',traj.p(idx,1),'YData',traj.p(idx,2),'ZData',traj.p(idx,3));
    set(h2_mk,'XData',traj.p(idx,1),'YData',traj.p(idx,2));

    % 估计
    if exist('h3_est','var')
        set(h3_est,'XData',traj.est.p(1:i,1),'YData',traj.est.p(1:i,2),'ZData',traj.est.p(1:i,3));
        set(h2_est,'XData',traj.est.p(1:i,1),'YData',traj.est.p(1:i,2));
    end

    uistack(h3_true,'top'); uistack(h2_true,'top');
    ax3_view = ax3_view + 0.5; view(ax_3d,[ax3_view, 15]);
    set(txt_time,'String',sprintf('Time(s): %.1f', traj.t(i)));
    drawnow;

    % === 添加 GIF 截图 ===
    genarate_gif(floor(traj.t(i)/Ts_disp));       % 将当前帧保存到 test.gif
    pause(0.01);         % 可选：防止过快绘制

end

end % ====== main ======


%% ---------- 基础轴属性 ----------
function baseAxes(ax)
view(ax,[30,10]); grid(ax,'on'); axis(ax,'equal');
xlabel(ax,'x'); ylabel(ax,'y'); zlabel(ax,'z');
end

%% ---------- 自动视野工具 ----------
function [ax3_x, ax3_y] = autoXY(traj, margin, squareXY, axisCenter)
mins = min(traj.p,[],1);  maxs = max(traj.p,[],1);
if ~isempty(axisCenter)
    cx = axisCenter(1); cy = axisCenter(2);
else
    cx = (mins(1)+maxs(1))/2; 
    cy = (mins(2)+maxs(2))/2;
end
halfX = (maxs(1)-mins(1))/2; 
halfY = (maxs(2)-mins(2))/2;
if squareXY
    R = max(halfX, halfY) * (1 + margin);
    ax3_x = [cx - R, cx + R];
    ax3_y = [cy - R, cy + R];
else
    ax3_x = [cx - halfX*(1+margin), cx + halfX*(1+margin)];
    ax3_y = [cy - halfY*(1+margin), cy + halfY*(1+margin)];
end
end

function ax3_z = autoZ(traj, zmargin)
mins = min(traj.p,[],1);  maxs = max(traj.p,[],1);
zmin = min(0, mins(3)); 
zmax = maxs(3);
zpad = zmargin*max(1, (zmax - zmin));
ax3_z = [zmin - zpad, zmax + zpad];
end

function [ax3_x, ax3_y, ax3_z] = autoAxesFromTraj(traj, opts)
[ax3_x, ax3_y] = autoXY(traj, opts.FitMargin, opts.SquareXY, opts.AxisCenter);
ax3_z = autoZ(traj, 0.1);
end

%% ---------- 本体几何建模（你的13个surface，不改样式） ----------
function combinedobj = build_os4_geometry_like_yours(ax, OS4_physical)
hold(ax,'on');
X_fl = [OS4_physical.size.fuselage.x/2 , OS4_physical.size.fuselage.x/2 , -OS4_physical.size.fuselage.x/2 , -OS4_physical.size.fuselage.x/2  OS4_physical.size.fuselage.x/2];
X_fl(2,:) = X_fl(1,:);
Y_fl = [-OS4_physical.size.fuselage.y/2 , OS4_physical.size.fuselage.y/2 , OS4_physical.size.fuselage.y/2 , -OS4_physical.size.fuselage.y/2, -OS4_physical.size.fuselage.y/2];
Y_fl(2,:) = Y_fl(1,:);
Z_fl = [-OS4_physical.size.fuselage.z/2 , -OS4_physical.size.fuselage.z/2 , -OS4_physical.size.fuselage.z/2 , -OS4_physical.size.fuselage.z/2 , -OS4_physical.size.fuselage.z/2];
Z_fl(2,:) = -Z_fl(1,:);
fuselage = surface(ax, X_fl, Y_fl, Z_fl, 'FaceColor',[0.8 0.8 0.8] );

[ Y_temp, Z_temp, X_temp ] = cylinder( OS4_physical.size.arm.diameter/2 );
ma_theta = zeros(1,4); ma_length = zeros(1,4);
X_ma = zeros([size(X_temp),4]); Y_ma = zeros([size(Y_temp),4]); Z_ma = zeros([size(Z_temp),4]);
for i = 1:4
    ma_theta(i)  = atan2( OS4_physical.rotor.pos(2,i) , OS4_physical.rotor.pos(1,i) );
    ma_length(i) = sqrt( OS4_physical.rotor.pos(2,i)^2 + OS4_physical.rotor.pos(1,i)^2 );
    X_ma(1,:,i) = X_temp(1,:)*ma_length(i)*cos(ma_theta(i)) - Y_temp(1,:)*sin(ma_theta(i));
    X_ma(2,:,i) = X_temp(2,:)*ma_length(i)*cos(ma_theta(i)) - Y_temp(2,:)*sin(ma_theta(i));
    Y_ma(1,:,i) = Y_temp(1,:)*cos(ma_theta(i)) + X_temp(1,:)*ma_length(i)*sin(ma_theta(i));
    Y_ma(2,:,i) = Y_temp(2,:)*cos(ma_theta(i)) + X_temp(2,:)*ma_length(i)*sin(ma_theta(i));
    Z_ma(1,:,i) = Z_temp(1,:);  Z_ma(2,:,i) = Z_temp(2,:);
end
ma = gobjects(1,4);
for i = 1:4
    ma(i) = surface(ax, X_ma(:,:,i), Y_ma(:,:,i), Z_ma(:,:,i) + OS4_physical.rotor.pos(3,i)+OS4_physical.size.motor.height,'FaceColor','k' );
end

[ X_mt, Y_mt, Z_mt ] = cylinder( OS4_physical.size.motor.diameter/2 );
motor = gobjects(1,4);
for i = 1:4
    motor(i) = surface(ax, X_mt + OS4_physical.rotor.pos(1,i), ...
                           Y_mt + OS4_physical.rotor.pos(2,i), ...
                         - Z_mt*OS4_physical.size.motor.height + OS4_physical.rotor.pos(3,i) + OS4_physical.size.motor.height );
end

t = 0:0.1:2*pi;
X_prop = [OS4_physical.prop.rad * sin(t), 0];
X_prop(2,:) = zeros(1,64);
Y_prop = [OS4_physical.prop.rad * cos(t), OS4_physical.prop.rad];
Y_prop(2,:) = zeros(1,64);
Z_prop = zeros(2,64);
propeller = gobjects(1,4);
i=1; propeller(i) = surface(ax, X_prop+OS4_physical.rotor.pos(1,i), Y_prop+OS4_physical.rotor.pos(2,i), Z_prop+OS4_physical.rotor.pos(3,i),'FaceColor','k','EdgeColor','none','LineStyle','none');
i=2; propeller(i) = surface(ax, X_prop+OS4_physical.rotor.pos(1,i), Y_prop+OS4_physical.rotor.pos(2,i), Z_prop+OS4_physical.rotor.pos(3,i),'FaceColor',[0.8,0.8,0.8],'EdgeColor','none','LineStyle','none');
i=3; propeller(i) = surface(ax, X_prop+OS4_physical.rotor.pos(1,i), Y_prop+OS4_physical.rotor.pos(2,i), Z_prop+OS4_physical.rotor.pos(3,i),'FaceColor',[0.8,0.8,0.8],'EdgeColor','none','LineStyle','none');
i=4; propeller(i) = surface(ax, X_prop+OS4_physical.rotor.pos(1,i), Y_prop+OS4_physical.rotor.pos(2,i), Z_prop+OS4_physical.rotor.pos(3,i),'FaceColor','k','EdgeColor','none','LineStyle','none');

combinedobj = gobjects(1,13);
combinedobj(1) = fuselage;
combinedobj(2:5) = ma(1:4);
combinedobj(6:9) = motor(1:4);
combinedobj(10:13) = propeller(1:4);
rotate(combinedobj,[1 1 0],180);
end

%% ---------- 欧拉角 -> 轴角 ----------
function [theta_b,rx,ry,rz]=Eular2axisangle(fai,theta,psi)
q0 =  cos(psi/2)*cos(theta/2)*cos(fai/2) + sin(psi/2)*sin(theta/2)*sin(fai/2);
q1 =  cos(psi/2)*cos(theta/2)*sin(fai/2) - sin(psi/2)*sin(theta/2)*cos(fai/2);
q2 =  cos(psi/2)*sin(theta/2)*cos(fai/2) + sin(psi/2)*cos(theta/2)*sin(fai/2);
q3 =  sin(psi/2)*cos(theta/2)*cos(fai/2) - cos(psi/2)*sin(theta/2)*sin(fai/2);
theta_b = 2*acos(q0);
if theta_b ~= 0
    rx = q1/sin(theta_b/2); ry = q2/sin(theta_b/2); rz = q3/sin(theta_b/2);
else
    rx = 1; ry = 0; rz = 0;
end
end

%% ================== 示例/占位轨迹（动力学姿态映射版） ==================

% --- 螺旋（动力学姿态） ---
function traj = demo_helix_dyn(p, Ts, g)
t = 0:Ts:p.Tend;
x = p.R*cos(p.omega*t);  y = p.R*sin(p.omega*t);  z = p.z0 + p.vz*t;
vx = -p.R*p.omega*sin(p.omega*t); vy =  p.R*p.omega*cos(p.omega*t); vz = p.vz + 0*t;
ax = -p.R*p.omega^2*cos(p.omega*t); ay = -p.R*p.omega^2*sin(p.omega*t); az = 0*t;
psi = atan2(vy, vx);                         % 沿切向航向
[phi,theta] = accel2euler_vec(ax,ay,az,psi,g);
traj.t = t; traj.p = [x.' y.' z.'];
traj.v = [vx.' vy.' vz.'];
traj.ax = ax.'; traj.ay = ay.'; traj.az = az.';
traj.euler = [phi, theta, psi.'];
end

% --- 第1章占位：z阶跃，姿态≈0 ---
function traj = demo_ch1_placeholder(p, Ts)
t = 0:Ts:p.Tend;
z0 = 0.2; zf = p.z_step; sigma = t/t(end);
S  = (10*sigma.^3 - 15*sigma.^4 + 6*sigma.^5);
z  = z0 + (zf - z0)*S;
x  = zeros(size(t)); y = zeros(size(t));
vx = gradient(x,Ts); vy=gradient(y,Ts); vz=gradient(z,Ts);
traj.t=t; traj.p=[x.' y.' z.']; traj.v=[vx.' vy.' vz.'];
traj.ax=zeros(numel(t),1); traj.ay=zeros(numel(t),1); traj.az=gradient(vz,Ts).';
traj.euler=zeros(numel(t),3);
end

% --- 第2章占位：圆轨迹 + 高度阶跃（动力学姿态） ---
function traj = demo_ch2_placeholder_dyn(p, Ts, g)
t = 0:Ts:p.Tend; R=p.circle_R; om=p.circle_omega;
x = R*(1-exp(-0.4*t)).*cos(om*t);  y = R*(1-exp(-0.4*t)).*sin(om*t);
z = p.z_ref*(1-exp(-0.6*t));
vx=gradient(x,Ts); vy=gradient(y,Ts); vz=gradient(z,Ts);
ax=gradient(vx,Ts); ay=gradient(vy,Ts); az=gradient(vz,Ts);
psi = atan2(vy, vx);
[phi,theta] = accel2euler_vec(ax,ay,az,psi,g);
traj.t=t; traj.p=[x.' y.' z.']; traj.v=[vx.' vy.' vz.'];
traj.ax=ax.'; traj.ay=ay.'; traj.az=az.';
traj.euler=[phi, theta, psi.'];
end

% --- 第3章占位：KF（真值=螺旋；估计=简单低通） ---
function traj = demo_ch3_kf_placeholder_dyn(p, Ts, g)
true = demo_helix_dyn(p, Ts, g); rng(1);
ax_meas = true.ax + p.na*randn(size(true.ax));
ay_meas = true.ay + p.na*randn(size(true.ay));
psi_m   = unwrap(true.euler(:,3) + p.ng*randn(size(true.euler(:,3))));
alpha=0.15;
ax_hat = filter(alpha,[1 -(1-alpha)], ax_meas);
ay_hat = filter(alpha,[1 -(1-alpha)], ay_meas);
az_hat = zeros(size(ax_hat));
[phi_hat,theta_hat] = accel2euler_vec(ax_hat,ay_hat,az_hat,psi_m,g);
v_hat = filter(alpha,[1 -(1-alpha)], true.v);
p_hat = filter(alpha,[1 -(1-alpha)], true.p);
traj = true;
traj.est.p = p_hat; traj.est.euler = [phi_hat theta_hat psi_m];
end

% --- 第4章占位：五次多项式软着陆（动力学姿态） ---
function traj = demo_ch4_lqr_placeholder(p, Ts, g)
t = 0:Ts:p.Tend; sigma = t/t(end);
x0=0.6; y0=-0.3; z0=p.z0; S = (10*sigma.^3 - 15*sigma.^4 + 6*sigma.^5);
x  = x0*(1-S) + p.xg*S;  y = y0*(1-S) + p.yg*S;  z = z0*(1-S);
vx=gradient(x,Ts); vy=gradient(y,Ts); vz=gradient(z,Ts);
ax=gradient(vx,Ts); ay=gradient(vy,Ts); az=gradient(vz,Ts);
psi = atan2(vy + 1e-6, vx + 1e-6);
[phi,theta] = accel2euler_vec(ax,ay,az,psi,g);
traj.t=t; traj.p=[x.' y.' z.']; traj.v=[vx.' vy.' vz.'];
traj.ax=ax.'; traj.ay=ay.'; traj.az=az.';
traj.euler=[phi, theta, psi.'];
end

%% ---------- 向量化：由加速度与航向得到 (phi,theta) ----------
function [phi,theta] = accel2euler_vec(ax,ay,az,psi,g)
n = numel(ax); phi=zeros(n,1); theta=zeros(n,1);
for k=1:n
    a = [ax(k); ay(k); az(k)];
    [ph,th] = accel2euler(a, psi(k), g);
    phi(k)=ph; theta(k)=th;
end
end

% 单点：a_des, yaw -> roll, pitch（几何控制常用映射）
function [phi,theta] = accel2euler(a_des, yaw, g)
F = a_des(:) + [0;0;g];                    % 合力方向（与总推力同向）
if norm(F) < 1e-9, F=[0;0;g]; end
zb = F / norm(F);                          % 机体 z 轴方向
xc = [cos(yaw); sin(yaw); 0];              % 期望航向参考
yb = cross(zb, xc);  n2=norm(yb);
if n2 < 1e-9
    xc = [cos(yaw+1e-3); sin(yaw+1e-3); 0];  % 退避
    yb = cross(zb, xc); n2=norm(yb);
end
yb = yb / n2;
xb = cross(yb, zb);
R  = [xb yb zb];                           % R = [x_b y_b z_b]
theta = asin(-R(3,1));                     % ZYX 提取
phi   = atan2(R(3,2), R(3,3));
end

%% ---------- 应用平面缩放并重算姿态 ----------
function traj = applyPathScale_dyn(traj, opts, initVal, g)
S = opts.PathScale;
if isempty(S) || S==1 || ~isfield(traj,'p') || isempty(traj.p), return; end
% 中心
if ~isempty(opts.AxisCenter)
    cx=opts.AxisCenter(1); cy=opts.AxisCenter(2);
elseif ~isempty(initVal)
    cx=initVal.x00; cy=initVal.y00;
else
    cx=mean(traj.p(:,1)); cy=mean(traj.p(:,2));
end
% 缩放位置
traj.p(:,1) = cx + S*(traj.p(:,1)-cx);
traj.p(:,2) = cy + S*(traj.p(:,2)-cy);
% 同步缩放速度/加速度平面分量
if isfield(traj,'v')  && ~isempty(traj.v),  traj.v(:,1:2)  = S*traj.v(:,1:2);  end
if isfield(traj,'ax') && ~isempty(traj.ax), traj.ax        = S*traj.ax;        end
if isfield(traj,'ay') && ~isempty(traj.ay), traj.ay        = S*traj.ay;        end
% 重新计算偏航与姿态
if isfield(traj,'v') && ~isempty(traj.v), psi = atan2(traj.v(:,2), traj.v(:,1));
else, psi = zeros(size(traj.p,1),1); end
if ~isfield(traj,'az') || isempty(traj.az), traj.az = zeros(size(traj.p,1),1); end
[phi,theta] = accel2euler_vec(traj.ax, traj.ay, traj.az, psi, g);
traj.euler = [phi, theta, psi];
end

%% ---------- 欧拉获取（若已生成则直接用） ----------
function eul = getEuler(traj, i)
if isfield(traj,'euler') && ~isempty(traj.euler)
    eul = traj.euler(i,:).';
else
    eul = [0;0;0];
end
end

function genarate_gif(i)
  F=getframe(gcf);%将动态图保存到test.gif
  I=frame2im(F);
  [I,map]=rgb2ind(I,256);
  if(i==1)
    imwrite(I,map,'test.gif','gif', 'Loopcount',inf,'DelayTime',0);
  else
    imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',0);
  end 
end