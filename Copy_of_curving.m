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
P.delta_max = deg2rad(39);

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

P.vx0   = 10;
P.vy0   = 0;
P.psi0  = 0;      % 后面自动覆盖
P.r0    = 0;
P.x0    = 0;      % 后面自动覆盖
P.y0    = 0;      % 后面自动覆盖

P.start_s      = 0;   % 沿道路从哪里开始
P.start_offset = 0;   % 相对道路中心线的横向偏置，0表示车放在中心线上

% LQR 权重默认值
P.qv    = 60;
P.qy    = 1600;
P.qpsi  = 450;
P.qvy   = 8;
P.qr    = 12;
P.ra    = 1;
P.rdel  = 20;

% 控制饱和
P.a_max     = 2.5;
P.a_min     = -3.0;
P.delta_max = deg2rad(39);

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

%% ===================== 分析与扰动设置 =====================
runBatchAnalysis = true;     % 是否自动做多组场景分析
runWeightSweep   = true;     % 是否扫描LQR权重看tradeoff

% 道路坡度扰动（弧度）
dist.gradeType   = 0;        % 0=无坡度, 1=常值坡, 2=分段坡, 3=正弦坡
dist.gradeValue  = deg2rad(4);   % 常值坡度，例如4度
dist.gradeStart  = 40;       % 分段坡起点 x
dist.gradeEnd    = 90;       % 分段坡终点 x
dist.gradeAmp    = deg2rad(3);   % 正弦坡幅值
dist.gradeWave   = 60;       % 正弦坡波长

% 参数摄动
dist.massScale = 1.0;        % 车辆质量缩放
dist.CfScale   = 1.0;        % 前轮侧偏刚度缩放
dist.CrScale   = 1.0;        % 后轮侧偏刚度缩放

% 速度参考变化（做step response用）
dist.useSpeedStep = false;
dist.stepTime     = 4.0;
dist.vxRef2       = 25;      % step后目标速度

% 曲率突变测试
dist.useCurvatureBoost = false;
dist.curvBoostStart    = 50;
dist.curvBoostEnd      = 90;
dist.curvBoostScale    = 1.3;

if useInteractiveInput
    P.vx_ref     = inputDefault('目标速度 vx_ref (m/s)', P.vx_ref);
    P.vx0        = inputDefault('初始纵向速度 vx0 (m/s)', P.vx0);
    P.lane_width = inputDefault('车道宽度 lane_width (m)', P.lane_width);
    P.T          = inputDefault('仿真总时间 T (s)', P.T);
    P.m          = inputDefault('车辆质量 m (kg)', P.m);

    road.type = inputDefault('道路类型(1直路,2正弦,3弯弯绕绕,4S型)', road.type);

    switch road.type
        case 1
            disp('已选择：直路');
        case 2
            disp('已选择：单正弦弯道 y = A*sin(2*pi*x/L)');
        case 3
            disp('已选择：弯弯绕绕 y = A1*sin(2*pi*x/L1)+A2*sin(2*pi*x/L2)');
        case 4
            disp('已选择：S型道路');
        otherwise
            warning('道路类型输入无效，自动改为 3（弯弯绕绕）');
            road.type = 3;
    end

    fprintf('\n------ 坡度输入 ------\n');
    dist.gradeType = input('坡度类型 (0无坡, 1常值坡) [默认=0]: ');
    if isempty(dist.gradeType), dist.gradeType = 0; end

    while ~ismember(dist.gradeType, [0, 1])
        disp('输入无效，请输入 0 或 1。');
        dist.gradeType = input('坡度类型 (0无坡, 1常值坡) [默认=0]: ');
        if isempty(dist.gradeType), dist.gradeType = 0; end
    end

    dist.gradeValue = 0;
    if dist.gradeType == 1
        tmp = input('常值坡度角 (deg，正=上坡，负=下坡) [默认=4]: ');
        if isempty(tmp), tmp = 4; end
        dist.gradeValue = deg2rad(tmp);
    end
end
% ---------- 根据道路起点自动对齐车辆初始位姿 ----------
[P.x0, P.y0, P.psi0] = autoAlignInitialPose(road, P.start_s, P.start_offset);

fprintf('\n自动设置初始位姿:\n');
fprintf('x0   = %.3f m\n', P.x0);
fprintf('y0   = %.3f m\n', P.y0);
fprintf('psi0 = %.3f deg\n', rad2deg(P.psi0));
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
ay_hist      = zeros(N,1);   % 横向加速度
ax_hist      = zeros(N,1);   % 纵向加速度
jerk_hist    = zeros(N,1);   % 纵向jerk
vxref_hist   = zeros(N,1);   % 记录变化后的参考速度
grade_hist   = zeros(N,1);   % 记录坡度

%% ===================== 5. 闭环仿真 =====================
g = 9.81;
a_prev = 0;
ay_prev = 0;

for k = 1:N
    t = (k-1)*P.dt;

    % ---------- 当前参考速度（支持step response） ----------
    vx_ref_now = P.vx_ref;
    if dist.useSpeedStep && t >= dist.stepTime
        vx_ref_now = dist.vxRef2;
    end

    % ---------- 当前道路参考 ----------
    [y_ref, dy_dx, ddy_dx2] = roadProfile(xpos, road);

    % 曲率突变/增强测试
    if dist.useCurvatureBoost && xpos >= dist.curvBoostStart && xpos <= dist.curvBoostEnd
        dy_dx   = dist.curvBoostScale * dy_dx;
        ddy_dx2 = dist.curvBoostScale * ddy_dx2;
    end

    psi_ref   = atan(dy_dx);
    kappa_ref = ddy_dx2 / (1 + dy_dx^2)^(3/2);

    % ---------- 当前坡度 ----------
    theta_grade = roadGradeProfile(xpos, dist);

    % ---------- 参数摄动 ----------
    m_now  = P.m  * dist.massScale;
    Iz_now = P.Iz * dist.massScale;   % 可简单同比缩放
    Cf_now = P.Cf * dist.CfScale;
    Cr_now = P.Cr * dist.CrScale;

    % ---------- 跟踪误差 ----------
    ev   = vx - vx_ref_now;
    ey   = ypos - y_ref;
    epsi = wrapToPiLocal(psi - psi_ref);

    x_state = [ev; ey; epsi; vy; r];

    % ---------- LQR反馈 ----------
    u_fb = -K * x_state;
    a_cmd = u_fb(1);

    % 曲率前馈
    delta_ff = atan((P.lf + P.lr) * kappa_ref);

    % 总转角
    delta_cmd = u_fb(2) + delta_ff;

    % 饱和
    a_cmd     = min(max(a_cmd, P.a_min), P.a_max);
    delta_cmd = min(max(delta_cmd, -P.delta_max), P.delta_max);

    % ---------- 非线性自行车模型 ----------
    vx_safe = max(vx, 0.5);

    alpha_f = delta_cmd - atan2(vy + P.lf*r, vx_safe);
    alpha_r = -atan2(vy - P.lr*r, vx_safe);

    Fyf = 2 * Cf_now * alpha_f;
    Fyr = 2 * Cr_now * alpha_r;

    % 加入坡度对纵向的影响
    vx_dot   = a_cmd - g * sin(theta_grade);
    vy_dot   = (Fyf + Fyr)/m_now - vx*r;
    r_dot    = (P.lf*Fyf - P.lr*Fyr)/Iz_now;
    psi_dot  = r;
    xpos_dot = vx*cos(psi) - vy*sin(psi);
    ypos_dot = vx*sin(psi) + vy*cos(psi);

    % comfort metrics
    ax_now = vx_dot;
    ay_now = vy_dot + vx*r;
    
    jerk_x_now = (a_cmd - a_prev) / P.dt;
    jerk_y_now = (ay_now - ay_prev) / P.dt;
    total_jerk_now = sqrt(jerk_x_now^2 + jerk_y_now^2);

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

    ax_hist(k)     = ax_now;
    ay_hist(k)     = ay_now;
    jerk_hist(k)   = total_jerk_now;
    vxref_hist(k)  = vx_ref_now;
    grade_hist(k)  = theta_grade;
    
    a_prev = a_cmd;
    ay_prev = ay_now;
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
metrics = computeMetrics(time_hist, vx_hist, vxref_hist, ey_hist, delta_hist, ...
                         ax_hist, ay_hist, jerk_hist, lane_bound);

fprintf('\n================ 仿真结果 ================\n');
fprintf('最终速度                     = %.3f m/s\n', vx_hist(end));
fprintf('最终速度误差                 = %.3f m/s\n', vx_hist(end)-vxref_hist(end));
fprintf('速度超调(相对最终参考)       = %.2f %%\n', metrics.speedOvershootPct);
fprintf('速度调节时间                 = %.3f s\n', metrics.speedSettlingTime);
fprintf('稳态速度误差                 = %.3f m/s\n', metrics.speedSteadyError);

fprintf('最大横向误差                 = %.3f m\n', metrics.maxEy);
fprintf('横向误差RMS                  = %.3f m\n', metrics.rmsEy);
fprintf('稳态横向误差                 = %.3f m\n', metrics.ssEy);

fprintf('最大前轮转角                 = %.3f deg\n', metrics.maxDeltaDeg);
fprintf('最大纵向加速度               = %.3f m/s^2\n', metrics.maxAx);
fprintf('最大横向加速度               = %.3f m/s^2\n', metrics.maxAy);
fprintf('最大总jerk                   = %.3f m/s^3\n', metrics.maxJerk);
fprintf('RMS总jerk                    = %.3f m/s^3\n', metrics.rmsJerk);
fprintf('是否始终保持在车道内         = %d\n', metrics.insideLane);

if metrics.insideLane
    disp('结论：车辆始终保持在道路边界内。');
else
    disp('结论：车辆存在越界情况，请降低车速或重新调整Q、R。');
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

%% ===================== 10. 舒适性分析图 =====================
figure('Color','w','Position',[180 120 1100 700]);

subplot(3,1,1);
plot(time_hist, ax_hist, 'LineWidth', 1.8); grid on;
xlabel('时间 (s)'); ylabel('a_x (m/s^2)');
title('纵向加速度');

subplot(3,1,2);
plot(time_hist, ay_hist, 'LineWidth', 1.8); grid on;
xlabel('时间 (s)'); ylabel('a_y (m/s^2)');
title('横向加速度');

subplot(3,1,3);
plot(time_hist, jerk_hist, 'LineWidth', 1.8); grid on;
xlabel('时间 (s)'); ylabel('jerk (m/s^3)');
title('Total jerk');

%% ===================== 11. 多场景鲁棒性分析 =====================
if runBatchAnalysis
    disp(' ');
    disp('============= 开始多场景分析 =============');

    scenarioList = {};

    % 1 baseline
    S.name = 'Baseline';
    S.dist = dist;
    S.road = road;
    scenarioList{end+1} = S;

    % 2 uphill
    S.name = 'Uphill +4deg';
    S.dist = dist; 
    S.dist.gradeType = 1;
    S.dist.gradeValue = deg2rad(4);
    S.road = road;
    scenarioList{end+1} = S;

    % 3 downhill
    S.name = 'Downhill -4deg';
    S.dist = dist;
    S.dist.gradeType = 1;
    S.dist.gradeValue = deg2rad(-4);
    S.road = road;
    scenarioList{end+1} = S;

    % 4 heavy vehicle
    S.name = 'Mass +20%';
    S.dist = dist;
    S.dist.massScale = 1.2;
    S.road = road;
    scenarioList{end+1} = S;

    % 5 light vehicle
    S.name = 'Mass -20%';
    S.dist = dist;
    S.dist.massScale = 0.8;
    S.road = road;
    scenarioList{end+1} = S;

    % 6 low tire stiffness
    S.name = 'CfCr -30%';
    S.dist = dist;
    S.dist.CfScale = 0.7;
    S.dist.CrScale = 0.7;
    S.road = road;
    scenarioList{end+1} = S;

    % 7 stronger curvature
    S.name = 'Curvature Boost';
    S.dist = dist;
    S.dist.useCurvatureBoost = true;
    S.dist.curvBoostStart = 45;
    S.dist.curvBoostEnd   = 100;
    S.dist.curvBoostScale = 2.0;
    S.road = road;
    scenarioList{end+1} = S;

    % 8 speed step
    S.name = 'Speed Step 20to25';
    S.dist = dist;
    S.dist.useSpeedStep = true;
    S.dist.stepTime = 4;
    S.dist.vxRef2 = 25;
    S.road = road;
    scenarioList{end+1} = S;

    resultNames = strings(length(scenarioList),1);
    maxEyVec    = zeros(length(scenarioList),1);
    rmsEyVec    = zeros(length(scenarioList),1);
    ssEvVec     = zeros(length(scenarioList),1);
    settleVec   = zeros(length(scenarioList),1);
    maxDeltaVec = zeros(length(scenarioList),1);
    jerkVec     = zeros(length(scenarioList),1);
    laneVec     = zeros(length(scenarioList),1);

    for i = 1:length(scenarioList)
        out = simulateScenario(P, scenarioList{i}.road, scenarioList{i}.dist);

        resultNames(i) = string(scenarioList{i}.name);
        maxEyVec(i)    = out.metrics.maxEy;
        rmsEyVec(i)    = out.metrics.rmsEy;
        ssEvVec(i)     = out.metrics.speedSteadyError;
        settleVec(i)   = out.metrics.speedSettlingTime;
        maxDeltaVec(i) = out.metrics.maxDeltaDeg;
        jerkVec(i)     = out.metrics.rmsJerk;
        laneVec(i)     = out.metrics.insideLane;
    end

    Tresult = table(resultNames, maxEyVec, rmsEyVec, ssEvVec, settleVec, ...
                    maxDeltaVec, jerkVec, laneVec, ...
        'VariableNames', {'Scenario','MaxEy_m','RMSEy_m','SpeedSSerr_mps', ...
                          'SpeedSettle_s','MaxDelta_deg','RMSJerk','InsideLane'});
    disp(Tresult);

    figure('Color','w','Position',[220 120 1100 650]);
    tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
    
    x = 1:length(resultNames);
    
    nexttile;
    plot(x, maxEyVec, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
    xticks(x); xticklabels(resultNames); xtickangle(30);
    ylabel('Max |e_y| (m)'); title('最大横向误差对比');
    
    nexttile;
    plot(x, settleVec, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
    xticks(x); xticklabels(resultNames); xtickangle(30);
    ylabel('Settling Time (s)'); title('速度调节时间对比');
    
    nexttile;
    plot(x, maxDeltaVec, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
    xticks(x); xticklabels(resultNames); xtickangle(30);
    ylabel('Max \delta (deg)'); title('最大转角对比');
    
    nexttile;
    plot(x, jerkVec, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
    xticks(x); xticklabels(resultNames); xtickangle(30);
    ylabel('RMS total jerk'); title('舒适性对比');
end

%% ===================== 12. LQR权重tradeoff分析 =====================
if runWeightSweep
    disp(' ');
    disp('============= 开始LQR权重扫描 =============');

    qy_list   = [100, 500, 1200];  % Lateral error weights (Low, Mid, High)
    rdel_list = [5, 15, 45];       % Steering effort weights (Low, Mid, High)

    cnt = 1;
    sweepName = strings(length(qy_list)*length(rdel_list),1);
    sweepMaxEy = zeros(length(sweepName),1);
    sweepJerk  = zeros(length(sweepName),1);
    sweepDelta = zeros(length(sweepName),1);

    for iq = 1:length(qy_list)
        for ir = 1:length(rdel_list)
            Ptmp = P;
            Ptmp.qy   = qy_list(iq);
            Ptmp.rdel = rdel_list(ir);

            out = simulateScenario(Ptmp, road, dist);

            sweepName(cnt)  = "qy=" + qy_list(iq) + ",rdel=" + rdel_list(ir);
            sweepMaxEy(cnt) = out.metrics.maxEy;
            sweepJerk(cnt)  = out.metrics.rmsJerk;
            sweepDelta(cnt) = out.metrics.maxDeltaDeg;
            cnt = cnt + 1;
        end
    end

    Tsweep = table(sweepName, sweepMaxEy, sweepJerk, sweepDelta, ...
        'VariableNames', {'Weights','MaxEy_m','RMSJerk','MaxDelta_deg'});
    disp(Tsweep);

    figure('Color','w','Position',[240 150 1000 520]);
    tiledlayout(1,3,'Padding','compact','TileSpacing','compact');
    
    x = 1:length(sweepName);
    
    % Subplot 1: Tracking Performance
    nexttile;
    plot(x, sweepMaxEy, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
    xticks(x);
    xticklabels(sweepName); % <--- Use the actual value names
    xtickangle(45);         % <--- Tilt them for readability
    ylabel('Max |e_y| (m)');
    title('跟踪性能 (Tracking Performance)');
    
    % Subplot 2: Comfort (Jerk)
    nexttile;
    plot(x, sweepJerk, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
    xticks(x);
    xticklabels(sweepName); % <--- Use the actual value names
    xtickangle(45);         % <--- Tilt them
    ylabel('RMS jerk');
    title('舒适性 (Comfort)');
    
    % Subplot 3: Control Effort (Steering)
    nexttile;
    plot(x, sweepDelta, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
    xticks(x);
    xticklabels(sweepName); % <--- Use the actual value names
    xtickangle(45);         % <--- Tilt them
    ylabel('Max \delta (deg)');
    title('控制激烈程度 (Control Effort)');
    
    disp('===== Weight Sweep Case Mapping =====');
    for i = 1:length(sweepName)
        fprintf('%2d : %s\n', i, sweepName(i));
    end
end

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
    switch road.type
        case 1
            % 直路
            y = 0;
            dy = 0;
            ddy = 0;

        case 2
            % 单正弦弯道
            w = 2*pi/road.L;
            y   = road.A * sin(w*x);
            dy  = road.A * w * cos(w*x);
            ddy = -road.A * w^2 * sin(w*x);

        case 3
            % 双正弦叠加
            w1 = 2*pi/road.L1;
            w2 = 2*pi/road.L2;
            y   = road.A1*sin(w1*x) + road.A2*sin(w2*x);
            dy  = road.A1*w1*cos(w1*x) + road.A2*w2*cos(w2*x);
            ddy = -road.A1*w1^2*sin(w1*x) - road.A2*w2^2*sin(w2*x);

        case 4
            % S型道路
            z1 = (x - road.x1)/road.w;
            z2 = (x - road.x2)/road.w;

            y = road.A*tanh(z1) - road.A*tanh(z2);
            dy = road.A*(sechLocal(z1)^2)/road.w ...
               - road.A*(sechLocal(z2)^2)/road.w;
            ddy = road.A*(-2*sechLocal(z1)^2*tanh(z1))/road.w^2 ...
                - road.A*(-2*sechLocal(z2)^2*tanh(z2))/road.w^2;

        otherwise
            y = 0;
            dy = 0;
            ddy = 0;
    end
end

function [x0, y0, psi0] = autoAlignInitialPose(road, s0, y_offset)
    [y_ref0, dy_ref0, ~] = roadProfile(s0, road);

    x0 = s0;
    y0 = y_ref0 + y_offset;
    psi0 = atan(dy_ref0);
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

function theta = roadGradeProfile(~, dist)
    if dist.gradeType == 1
        theta = dist.gradeValue;
    else
        theta = 0;
    end
end

function metrics = computeMetrics(t, vx, vxref, ey, delta, ax, ay, jerk, lane_bound)
    speed_err = vx - vxref;
    final_ref = vxref(end);

    if abs(final_ref) < 1e-6
        speedOvershootPct = 0;
    else
        speedOvershootPct = max(0, (max(vx)-final_ref)/abs(final_ref)*100);
    end

    band = 0.02 * max(abs(final_ref), 1);   % 2% band
    idx_settle = find(abs(speed_err) <= band, 1, 'first');
    if isempty(idx_settle)
        settlingTime = NaN;
    else
        % 更严格：从该点后一直在band内
        settlingTime = NaN;
        for i = idx_settle:length(t)
            if all(abs(speed_err(i:end)) <= band)
                settlingTime = t(i);
                break;
            end
        end
    end

    nTail = max(10, round(0.1*length(t)));

    metrics.speedOvershootPct = speedOvershootPct;
    metrics.speedSettlingTime = settlingTime;
    metrics.speedSteadyError  = mean(speed_err(end-nTail+1:end));

    nSkip = max(1, round(0.15*length(ey)));   % skip first 15%
    ey_eval = ey(nSkip:end);
    
    metrics.maxEy      = max(abs(ey));
    metrics.maxEyAfter = max(abs(ey_eval));
    metrics.rmsEy      = sqrt(mean(ey_eval.^2));
    metrics.ssEy       = mean(ey(end-nTail+1:end));
    metrics.rmsEy  = sqrt(mean(ey.^2));
    metrics.ssEy   = mean(ey(end-nTail+1:end));

% 1. Find when the vehicle first reaches the target speed (within 2% band)
    band = 0.02 * max(abs(final_ref), 1);
    idx_enter = find(abs(speed_err) <= band, 1, 'first');
    
    % Fallback: if it never reaches target speed, evaluate the whole array
    if isempty(idx_enter)
        idx_enter = 1; 
    end

    % 2. Calculate Control & Comfort Metrics ONLY after reaching reference speed
    delta_eval = delta(idx_enter:end);
    ax_eval    = ax(idx_enter:end);
    ay_eval    = ay(idx_enter:end);
    jerk_eval  = jerk(idx_enter:end);

    metrics.maxDeltaDeg = max(abs(rad2deg(delta_eval)));
    metrics.maxAx       = max(abs(ax_eval));
    metrics.maxAy       = max(abs(ay_eval));
    metrics.maxJerk     = max(abs(jerk_eval));
    % 找到进入速度容差范围的索引
    idx_enter = find(abs(vx - vxref) <= band, 1, 'first');
    
    % 只取进入稳态后的 jerk 进行计算
    jerk_eval = jerk(idx_enter:end);
    metrics.rmsJerk = sqrt(mean(jerk_eval.^2));

    metrics.insideLane = all(abs(ey) <= lane_bound);
end

function out = simulateScenario(P, road, dist)
    lane_bound = P.lane_width/2;
    N = round(P.T/P.dt);
    g = 9.81;

    % LQR model
    V = P.vx_ref;
    A = [ 0, 0, 0, 0, 0;
          0, 0, V, 1, 0;
          0, 0, 0, 0, 1;
          0, 0, 0, -(2*P.Cf+2*P.Cr)/(P.m*V), -(V + (2*P.Cf*P.lf-2*P.Cr*P.lr)/(P.m*V));
          0, 0, 0, -(2*P.Cf*P.lf-2*P.Cr*P.lr)/(P.Iz*V), -(2*P.Cf*P.lf^2+2*P.Cr*P.lr^2)/(P.Iz*V)];

    B = [1, 0;
         0, 0;
         0, 0;
         0, 2*P.Cf/P.m;
         0, 2*P.Cf*P.lf/P.Iz];

    Q = diag([P.qv, P.qy, P.qpsi, P.qvy, P.qr]);
    R = diag([P.ra, P.rdel]);
    K = lqr(A, B, Q, R);

    % init
    vx   = P.vx0;
    vy   = P.vy0;
    psi  = P.psi0;
    r    = P.r0;
    xpos = P.x0;
    ypos = P.y0;
    a_prev = 0;
    ay_prev = 0;

    time_hist  = zeros(N,1);
    vx_hist    = zeros(N,1);
    ey_hist    = zeros(N,1);
    delta_hist = zeros(N,1);
    ax_hist    = zeros(N,1);
    ay_hist    = zeros(N,1);
    jerk_hist  = zeros(N,1);
    vxref_hist = zeros(N,1);

    for k = 1:N
        t = (k-1)*P.dt;

        vx_ref_now = P.vx_ref;
        if dist.useSpeedStep && t >= dist.stepTime
            vx_ref_now = dist.vxRef2;
        end

        [y_ref, dy_dx, ddy_dx2] = roadProfile(xpos, road);

        if dist.useCurvatureBoost && xpos >= dist.curvBoostStart && xpos <= dist.curvBoostEnd
            dy_dx   = dist.curvBoostScale * dy_dx;
            ddy_dx2 = dist.curvBoostScale * ddy_dx2;
        end

        psi_ref   = atan(dy_dx);
        kappa_ref = ddy_dx2 / (1 + dy_dx^2)^(3/2);

        theta_grade = roadGradeProfile(xpos, dist);

        m_now  = P.m  * dist.massScale;
        Iz_now = P.Iz * dist.massScale;
        Cf_now = P.Cf * dist.CfScale;
        Cr_now = P.Cr * dist.CrScale;

        ev   = vx - vx_ref_now;
        ey   = ypos - y_ref;
        epsi = wrapToPiLocal(psi - psi_ref);
        x_state = [ev; ey; epsi; vy; r];

        u_fb = -K*x_state;
        a_cmd = min(max(u_fb(1), P.a_min), P.a_max);

        delta_ff = atan((P.lf + P.lr) * kappa_ref);
        delta_cmd = min(max(u_fb(2) + delta_ff, -P.delta_max), P.delta_max);

        vx_safe = max(vx, 0.5);
        alpha_f = delta_cmd - atan2(vy + P.lf*r, vx_safe);
        alpha_r = -atan2(vy - P.lr*r, vx_safe);

        Fyf = 2 * Cf_now * alpha_f;
        Fyr = 2 * Cr_now * alpha_r;

        vx_dot = a_cmd - g*sin(theta_grade);
        vy_dot = (Fyf + Fyr)/m_now - vx*r;
        r_dot  = (P.lf*Fyf - P.lr*Fyr)/Iz_now;

        ax_now   = vx_dot;
        ay_now   = vy_dot + vx*r;
        jerk_now = (a_cmd - a_prev)/P.dt;
        ay_jerk = (ay_now - ay_prev) / P.dt;
        total_jerk = sqrt(jerk_now^2 + ay_jerk^2);

        psi_dot  = r;
        xpos_dot = vx*cos(psi) - vy*sin(psi);
        ypos_dot = vx*sin(psi) + vy*cos(psi);

        vx   = vx   + vx_dot*P.dt;
        vy   = vy   + vy_dot*P.dt;
        r    = r    + r_dot*P.dt;
        psi  = psi  + psi_dot*P.dt;
        xpos = xpos + xpos_dot*P.dt;
        ypos = ypos + ypos_dot*P.dt;

        time_hist(k)  = t;
        vx_hist(k)    = vx;
        ey_hist(k)    = ey;
        delta_hist(k) = delta_cmd;
        ax_hist(k)    = ax_now;
        ay_hist(k)    = ay_now;
        jerk_hist(k)  = total_jerk;
        vxref_hist(k) = vx_ref_now;

        a_prev = a_cmd;
        ay_prev = ay_now;
    end

    out.metrics = computeMetrics(time_hist, vx_hist, vxref_hist, ey_hist, delta_hist, ...
                                 ax_hist, ay_hist, jerk_hist, lane_bound);
end
