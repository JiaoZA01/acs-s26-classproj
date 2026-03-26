clc; clear; close all;

%% =========================================================
%  LQR 定速巡航 + 弯曲道路车道保持（可交互输入参数）
%  状态: x = [ev; ey; epsi; vy; r]
%  控制: u = [a; delta]
%  ev   = vx - vx_ref
%  ey   = y - y_ref(x)
%  epsi = psi - psi_ref(x)
%% =========================================================

disp('==============================================');
disp('   LQR 定速巡航 + 弯曲道路车道保持仿真');
disp('==============================================');
disp('道路类型可选:');
disp('1 - 直路');
disp('2 - 单正弦弯道');
disp('3 - 弯弯绕绕(双正弦叠加)');
disp('4 - S型道路');
disp('----------------------------------------------');

%% ===================== 1. 用户输入参数 =====================
% 是否交互输入
useInteractiveInput = true;

% -------- 默认参数 --------
P.m  = 1500;        % kg
P.Iz = 3000;        % kg*m^2
P.lf = 1.2;         % m
P.lr = 1.6;         % m
P.Cf = 8e4;         % N/rad
P.Cr = 8e4;         % N/rad

P.vx_ref     = 20;      % m/s
P.lane_width = 3.5;     % m
P.dt         = 0.01;    % s
P.T          = 16;      % s

P.vx0   = 10;           % 初始纵向速度
P.vy0   = 0.2;          % 初始侧向速度
P.psi0  = deg2rad(8);   % 初始航向角(弧度)
P.r0    = 0;            % 初始横摆角速度
P.x0    = 0;            % 初始x
P.y0    = 1.2;          % 初始y

% LQR 权重默认值
P.qv    = 60;
P.qy    = 260;
P.qpsi  = 160;
P.qvy   = 8;
P.qr    = 12;
P.ra    = 1;
P.rdel  = 6;

% 控制饱和
P.a_max     = 2.5;
P.a_min     = -3.0;
P.delta_max = deg2rad(20);

% 道路默认
road.type = 3;      % 默认弯弯绕绕
road.A1   = 1.2;
road.L1   = 70;
road.A2   = 0.5;
road.L2   = 25;
road.A    = 1.5;
road.L    = 80;
road.x1   = 35;
road.x2   = 90;
road.w    = 12;

if useInteractiveInput
    P.vx_ref     = inputDefault('目标速度 vx_ref (m/s)', P.vx_ref);
    P.lane_width = inputDefault('车道宽度 lane_width (m)', P.lane_width);
    P.T          = inputDefault('仿真总时间 T (s)', P.T);
    P.dt         = inputDefault('积分步长 dt (s)', P.dt);

    P.vx0   = inputDefault('初始纵向速度 vx0 (m/s)', P.vx0);
    P.vy0   = inputDefault('初始侧向速度 vy0 (m/s)', P.vy0);
    P.y0    = inputDefault('初始横向位置 y0 (m)', P.y0);
    psi0deg = inputDefault('初始航向角 psi0 (deg)', rad2deg(P.psi0));
    P.psi0  = deg2rad(psi0deg);

    road.type = inputDefault('道路类型(1直路,2正弦,3弯弯绕绕,4S型)', road.type);

    switch road.type
        case 1
            disp('已选择：直路');
        case 2
            disp('已选择：单正弦弯道 y = A*sin(2*pi*x/L)');
            road.A = inputDefault('正弦振幅 A (m)', road.A);
            road.L = inputDefault('正弦波长 L (m)', road.L);
        case 3
            disp('已选择：弯弯绕绕 y = A1*sin(2*pi*x/L1)+A2*sin(2*pi*x/L2)');
            road.A1 = inputDefault('A1 (m)', road.A1);
            road.L1 = inputDefault('L1 (m)', road.L1);
            road.A2 = inputDefault('A2 (m)', road.A2);
            road.L2 = inputDefault('L2 (m)', road.L2);
        case 4
            disp('已选择：S型道路');
            road.A  = inputDefault('横向变道幅值 A (m)', road.A);
            road.x1 = inputDefault('第一弯中心 x1 (m)', road.x1);
            road.x2 = inputDefault('第二弯中心 x2 (m)', road.x2);
            road.w  = inputDefault('弯道平滑宽度 w (m)', road.w);
        otherwise
            warning('道路类型输入无效，自动改为 3（弯弯绕绕）');
            road.type = 3;
    end

    disp('------ LQR 权重输入（越大越重视）------');
    P.qv   = inputDefault('Q中速度误差权重 qv', P.qv);
    P.qy   = inputDefault('Q中横向误差权重 qy', P.qy);
    P.qpsi = inputDefault('Q中航向误差权重 qpsi', P.qpsi);
    P.qvy  = inputDefault('Q中侧向速度权重 qvy', P.qvy);
    P.qr   = inputDefault('Q中横摆角速度权重 qr', P.qr);

    P.ra   = inputDefault('R中加速度权重 ra', P.ra);
    P.rdel = inputDefault('R中转角权重 rdel', P.rdel);

    P.a_max = inputDefault('最大加速度 a_max (m/s^2)', P.a_max);
    P.a_min = inputDefault('最小加速度 a_min (m/s^2)', P.a_min);
    deltaDeg = inputDefault('最大前轮转角 delta_max (deg)', rad2deg(P.delta_max));
    P.delta_max = deg2rad(deltaDeg);
end

lane_bound = P.lane_width/2;
N = round(P.T/P.dt);

%% ===================== 2. LQR 线性模型 =====================
V = P.vx_ref;

A = [ 0,                          0,                 0,                              0,                                0;
      0,                          0,                 V,                              1,                                0;
      0,                          0,                 0,                              0,                                1;
      0,                          0,                 0,  -(2*P.Cf+2*P.Cr)/(P.m*V),  -(V + (2*P.Cf*P.lf-2*P.Cr*P.lr)/(P.m*V));
      0,                          0,                 0,  -(2*P.Cf*P.lf-2*P.Cr*P.lr)/(P.Iz*V), ...
                                                      -(2*P.Cf*P.lf^2+2*P.Cr*P.lr^2)/(P.Iz*V)];

B = [1,          0;
     0,          0;
     0,          0;
     0,      2*P.Cf/P.m;
     0,  2*P.Cf*P.lf/P.Iz];

Q = diag([P.qv, P.qy, P.qpsi, P.qvy, P.qr]);
R = diag([P.ra, P.rdel]);

K = lqr(A, B, Q, R);

disp('----------------------------------------------');
disp('LQR 增益 K = ');
disp(K);

%% ===================== 3. 初始状态 =====================
vx   = P.vx0;
vy   = P.vy0;
psi  = P.psi0;
r    = P.r0;
xpos = P.x0;
ypos = P.y0;

%% ===================== 4. 数据记录 =====================
time_hist   = zeros(N,1);
x_hist      = zeros(N,1);
y_hist      = zeros(N,1);
vx_hist     = zeros(N,1);
vy_hist     = zeros(N,1);
psi_hist    = zeros(N,1);
r_hist      = zeros(N,1);
ey_hist     = zeros(N,1);
epsi_hist   = zeros(N,1);
a_hist      = zeros(N,1);
delta_hist  = zeros(N,1);
yref_hist   = zeros(N,1);
psiref_hist = zeros(N,1);
kappa_hist  = zeros(N,1);

%% ===================== 5. 闭环仿真 =====================
for k = 1:N
    t = (k-1)*P.dt;

    % 当前道路参考
    [y_ref, dy_dx, ddy_dx2] = roadProfile(xpos, road);
    psi_ref = atan(dy_dx);
    kappa_ref = ddy_dx2 / (1 + dy_dx^2)^(3/2);

    % 跟踪误差
    ev   = vx - P.vx_ref;
    ey   = ypos - y_ref;
    epsi = wrapToPiLocal(psi - psi_ref);

    x_state = [ev; ey; epsi; vy; r];

    % LQR反馈
    u_fb = -K * x_state;
    a_cmd = u_fb(1);

    % 曲率前馈转向（简化版本）
    delta_ff = atan((P.lf + P.lr) * kappa_ref);

    % 总转角
    delta_cmd = u_fb(2) + delta_ff;

    % 饱和
    a_cmd     = min(max(a_cmd, P.a_min), P.a_max);
    delta_cmd = min(max(delta_cmd, -P.delta_max), P.delta_max);

    % 非线性自行车模型
    vx_safe = max(vx, 0.5);

    alpha_f = delta_cmd - atan2(vy + P.lf*r, vx_safe);
    alpha_r = -atan2(vy - P.lr*r, vx_safe);

    Fyf = 2 * P.Cf * alpha_f;
    Fyr = 2 * P.Cr * alpha_r;

    vx_dot   = a_cmd;
    vy_dot   = (Fyf + Fyr)/P.m - vx*r;
    r_dot    = (P.lf*Fyf - P.lr*Fyr)/P.Iz;
    psi_dot  = r;
    xpos_dot = vx*cos(psi) - vy*sin(psi);
    ypos_dot = vx*sin(psi) + vy*cos(psi);

    % 欧拉积分
    vx   = vx   + vx_dot*P.dt;
    vy   = vy   + vy_dot*P.dt;
    r    = r    + r_dot*P.dt;
    psi  = psi  + psi_dot*P.dt;
    xpos = xpos + xpos_dot*P.dt;
    ypos = ypos + ypos_dot*P.dt;

    % 记录
    time_hist(k)   = t;
    x_hist(k)      = xpos;
    y_hist(k)      = ypos;
    vx_hist(k)     = vx;
    vy_hist(k)     = vy;
    psi_hist(k)    = psi;
    r_hist(k)      = r;
    ey_hist(k)     = ey;
    epsi_hist(k)   = epsi;
    a_hist(k)      = a_cmd;
    delta_hist(k)  = delta_cmd;
    yref_hist(k)   = y_ref;
    psiref_hist(k) = psi_ref;
    kappa_hist(k)  = kappa_ref;
end

%% ===================== 6. 生成道路用于绘图 =====================
x_plot_max = max(x_hist) + 20;
x_road = linspace(0, x_plot_max, 1500);
y_road = zeros(size(x_road));
for i = 1:length(x_road)
    [y_road(i), ~, ~] = roadProfile(x_road(i), road);
end
y_up = y_road + lane_bound;
y_dn = y_road - lane_bound;

%% ===================== 7. 动画显示 =====================
car_L = 4.5;
car_W = 1.8;

fig = figure('Color','w','Position',[80 60 1200 760]);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

% ---------- 左上：道路与车辆动画 ----------
ax1 = nexttile(1,[2 1]);
hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
xlabel(ax1,'x (m)');
ylabel(ax1,'y (m)');
title(ax1,'车辆在弯曲道路中的跟踪动画');

plot(ax1, x_road, y_up, '--', 'LineWidth', 1.2);
plot(ax1, x_road, y_dn, '--', 'LineWidth', 1.2);
plot(ax1, x_road, y_road, '-.', 'LineWidth', 1.0);

trajLine = plot(ax1, x_hist(1), y_hist(1), 'LineWidth', 2);
[Xcar, Ycar] = getCarShape(x_hist(1), y_hist(1), psi_hist(1), car_L, car_W);
carPatch = patch(ax1, Xcar, Ycar, [0.2 0.6 0.9], 'FaceAlpha', 0.85);
headLine = plot(ax1, ...
    [x_hist(1), x_hist(1)+car_L/2*cos(psi_hist(1))], ...
    [y_hist(1), y_hist(1)+car_L/2*sin(psi_hist(1))], 'r', 'LineWidth', 2);

infoText = text(ax1, x_hist(1)+2, y_hist(1)+2, '', 'FontSize', 11, 'FontWeight', 'bold');

xlim(ax1, [0, 60]);
ylim(ax1, [min(y_dn)-2, max(y_up)+2]);

% ---------- 右上：速度 ----------
ax2 = nexttile(2);
hold(ax2,'on'); grid(ax2,'on');
xlabel(ax2,'时间 (s)');
ylabel(ax2,'速度 (m/s)');
title(ax2,'定速巡航过程');
plot(ax2, time_hist, P.vx_ref*ones(size(time_hist)), '--r', 'LineWidth', 1.5);
speedLine = plot(ax2, time_hist(1), vx_hist(1), 'LineWidth', 2);
legend(ax2,'目标速度','实际速度','Location','southeast');
xlim(ax2,[0 P.T]);
ylim(ax2,[0, max(max(vx_hist)+2, P.vx_ref+2)]);

% ---------- 右中：横向误差 ----------
ax3 = nexttile(4);
hold(ax3,'on'); grid(ax3,'on');
xlabel(ax3,'时间 (s)');
ylabel(ax3,'e_y (m)');
title(ax3,'横向误差');
eyLine = plot(ax3, time_hist(1), ey_hist(1), 'LineWidth', 2);
yline(ax3, lane_bound, '--r', 'LineWidth', 1.0);
yline(ax3, -lane_bound, '--r', 'LineWidth', 1.0);
xlim(ax3,[0 P.T]);

% ---------- 单独再开一个窗口看转角 ----------
fig2 = figure('Color','w','Position',[180 120 900 380]);
hold on; grid on;
xlabel('时间 (s)');
ylabel('前轮转角 \delta (deg)');
title('转向控制输入');
deltaLine = plot(time_hist(1), rad2deg(delta_hist(1)), 'LineWidth', 2);
xlim([0 P.T]);

drawStep = 5;

for k = 1:drawStep:N
    % 更新左图可视窗口
    x_now = x_hist(k);
    idxWin = abs(x_road - x_now) < 35;
    if any(idxWin)
        y_min = min(y_dn(idxWin)) - 2;
        y_max = max(y_up(idxWin)) + 2;
    else
        y_min = min(y_dn) - 2;
        y_max = max(y_up) + 2;
    end

    if x_now > 30
        xlim(ax1, [x_now-30, x_now+30]);
    end
    ylim(ax1, [y_min, y_max]);

    % 更新车辆轨迹
    set(trajLine, 'XData', x_hist(1:k), 'YData', y_hist(1:k));

    % 更新车辆外形
    [Xcar, Ycar] = getCarShape(x_hist(k), y_hist(k), psi_hist(k), car_L, car_W);
    set(carPatch, 'XData', Xcar, 'YData', Ycar);

    % 更新车头方向
    set(headLine, ...
        'XData', [x_hist(k), x_hist(k)+car_L/2*cos(psi_hist(k))], ...
        'YData', [y_hist(k), y_hist(k)+car_L/2*sin(psi_hist(k))]);

    % 更新文字
    str = sprintf(['t = %.2f s\n' ...
                   'v = %.2f m/s\n' ...
                   'v_{ref} = %.2f m/s\n' ...
                   'e_y = %.2f m\n' ...
                   'e_{\\psi} = %.2f deg\n' ...
                   '\\delta = %.2f deg'], ...
                   time_hist(k), vx_hist(k), P.vx_ref, ey_hist(k), ...
                   rad2deg(epsi_hist(k)), rad2deg(delta_hist(k)));
    set(infoText, 'Position', [x_hist(k)+2, y_hist(k)+2, 0], 'String', str);

    % 更新速度和误差曲线
    set(speedLine, 'XData', time_hist(1:k), 'YData', vx_hist(1:k));
    set(eyLine, 'XData', time_hist(1:k), 'YData', ey_hist(1:k));

    % 更新转角图
    figure(fig2);
    set(deltaLine, 'XData', time_hist(1:k), 'YData', rad2deg(delta_hist(1:k)));

    drawnow;
    figure(fig);
end

%% ===================== 8. 仿真结果输出 =====================
inside_lane = all(abs(ey_hist) <= lane_bound);

fprintf('\n================ 仿真结果 ================\n');
fprintf('最终速度           = %.3f m/s\n', vx_hist(end));
fprintf('目标速度           = %.3f m/s\n', P.vx_ref);
fprintf('最终横向误差       = %.3f m\n', ey_hist(end));
fprintf('最大横向误差       = %.3f m\n', max(abs(ey_hist)));
fprintf('最大前轮转角       = %.3f deg\n', max(abs(rad2deg(delta_hist))));
fprintf('是否始终保持在车道内 = %d\n', inside_lane);

if inside_lane
    disp('结论：车辆始终保持在道路边界内。');
else
    disp('结论：车辆存在越界情况，请降低车速或重新调整 Q、R。');
end

%% ===================== 9. 静态结果图 =====================
figure('Color','w','Position',[100 80 1100 700]);

subplot(2,2,1);
plot(x_road, y_up, '--', 'LineWidth', 1.0); hold on;
plot(x_road, y_dn, '--', 'LineWidth', 1.0);
plot(x_road, y_road, '-.', 'LineWidth', 1.0);
plot(x_hist, y_hist, 'LineWidth', 2);
grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)');
title('车辆轨迹与道路');
legend('上边界','下边界','道路中心线','车辆轨迹');

subplot(2,2,2);
plot(time_hist, vx_hist, 'LineWidth', 2); hold on;
yline(P.vx_ref, '--r', 'LineWidth', 1.2);
grid on;
xlabel('时间 (s)');
ylabel('速度 (m/s)');
title('速度响应');

subplot(2,2,3);
plot(time_hist, ey_hist, 'LineWidth', 2); hold on;
yline(lane_bound, '--r');
yline(-lane_bound, '--r');
grid on;
xlabel('时间 (s)');
ylabel('横向误差 e_y (m)');
title('横向误差');

subplot(2,2,4);
plot(time_hist, rad2deg(delta_hist), 'LineWidth', 2); hold on;
grid on;
xlabel('时间 (s)');
ylabel('转角 \delta (deg)');
title('前轮转角输入');

%% ===================== 局部函数 =====================
function val = inputDefault(promptStr, defaultVal)
    str = input(sprintf('%s [默认=%.4g]: ', promptStr, defaultVal), 's');
    if isempty(str)
        val = defaultVal;
    else
        val = str2double(str);
        if isnan(val)
            val = defaultVal;
        end
    end
end

function [y, dy, ddy] = roadProfile(x, road)
    % 道路中心线写成 y = f(x)
    switch road.type
        case 1
            % 直路
            y = 0;
            dy = 0;
            ddy = 0;

        case 2
            % 单正弦
            % y = A*sin(2*pi*x/L)
            w = 2*pi/road.L;
            y   = road.A * sin(w*x);
            dy  = road.A * w * cos(w*x);
            ddy = -road.A * w^2 * sin(w*x);

        case 3
            % 弯弯绕绕：双正弦叠加
            % y = A1*sin(2*pi*x/L1) + A2*sin(2*pi*x/L2)
            w1 = 2*pi/road.L1;
            w2 = 2*pi/road.L2;
            y   = road.A1*sin(w1*x) + road.A2*sin(w2*x);
            dy  = road.A1*w1*cos(w1*x) + road.A2*w2*cos(w2*x);
            ddy = -road.A1*w1^2*sin(w1*x) - road.A2*w2^2*sin(w2*x);

        case 4
            % S型道路
            % y = A*tanh((x-x1)/w) - A*tanh((x-x2)/w)
            z1 = (x-road.x1)/road.w;
            z2 = (x-road.x2)/road.w;

            y = road.A*tanh(z1) - road.A*tanh(z2);

            dy = road.A*(sechLocal(z1)^2)/road.w ...
               - road.A*(sechLocal(z2)^2)/road.w;

            ddy = road.A*(-2*sechLocal(z1)^2*tanh(z1))/road.w^2 ...
                - road.A*(-2*sechLocal(z2)^2*tanh(z2))/road.w^2;

        otherwise
            y = 0; dy = 0; ddy = 0;
    end
end

function s = sechLocal(x)
    s = 1 ./ cosh(x);
end

function ang = wrapToPiLocal(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end

function [Xg, Yg] = getCarShape(xc, yc, psi, L, W)
    X = [ L/2,  L/2, -L/2, -L/2];
    Y = [ W/2, -W/2, -W/2,  W/2];

    R = [cos(psi), -sin(psi);
         sin(psi),  cos(psi)];

    P = R * [X; Y];
    Xg = P(1,:) + xc;
    Yg = P(2,:) + yc;
end