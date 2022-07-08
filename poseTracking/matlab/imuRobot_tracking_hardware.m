clc;

%% 获取imu实时数据绘制
path = "../data/SE3_imuData.csv";

dataImu = load(path);

x = dataImu(:,1);
y = dataImu(:,2);
z = dataImu(:,3);

qx = dataImu(:,4);
qy = dataImu(:,5);
qz = dataImu(:,6);
qw = dataImu(:,7);
tt = dataImu(:,8);

% 遍历每一个数据构造SO3
figure;
hold on;
axis equal;
T_v = [];
for i = 1:1:size(x,1)
   % 构造SE3 
   t = [x(i), y(i), z(i)];
   quat = quaternion(qw(i), qx(i), qy(i), qz(i));
   T = SE3(rotmat(quat,'point'), t);
   T_v = [T_v, T];

   plotframe(T,0.05);
end

%% 获取实时下发数据
path = "../data/SE3_P_control_path.csv";

dataTrack = load(path);

xc = dataTrack(:,1);
yc = dataTrack(:,2);
zc = dataTrack(:,3);

qxc = dataTrack(:,4);
qyc = dataTrack(:,5);
qzc = dataTrack(:,6);
qwc = dataTrack(:,7);
angvc = dataTrack(:,8);
ttc = dataTrack(:,9);

% 遍历每一个数据构造SO3
figure;
hold on;
axis equal;
T_vc = [];
for i = 1:1:size(xc,1)
   % 构造SE3 
   t = [xc(i), yc(i), zc(i)];
   quat = quaternion(qwc(i), qxc(i), qyc(i), qzc(i));
   T = SE3(rotmat(quat,'point'), t);
   T_vc = [T_vc, T];

   plotframe(T,0.05);
end

%% 获取机器人实时数据

path = "../data/SE3_P_control_robot_path.csv";

dataTrack = load(path);

xp = dataTrack(:,1);
yp = dataTrack(:,2);
zp = dataTrack(:,3);

qxp = dataTrack(:,4);
qyp = dataTrack(:,5);
qzp = dataTrack(:,6);
qwp = dataTrack(:,7);
angvp = dataTrack(:,8);
ttp = dataTrack(:,9);

% 遍历每一个数据构造SO3
figure;
hold on;
axis equal;
T_vp = [];
for i = 1:1:size(xp,1)
   % 构造SE3 
   t = [xp(i), yp(i), zp(i)];
   quat = quaternion(qwp(i), qxp(i), qyp(i), qzp(i));
   T = SE3(rotmat(quat,'point'), t);
   T_vp = [T_vp, T];

   plotframe(T,0.05);
end

%% 绘制误差曲线图

% 计算位置误差， 计算姿态角度误差

% 计算参数坐标误差
% err_x = x(1:end) - xp;
% err_y = y(1:end) - yp;
% err_z = z(1:end) - zp;W

ang_v = [];
% 计算imu姿态角误差
for i = 1:1:size(T_vp,2)
    [theta, w] = trlog(T_v(i).R'*T_vp(i).R);
    ang_v = [ang_v, theta];
end


ang_vc = [];
% 计算姿态角误差
for i = 1:1:size(T_vc,2)
    [theta, w] = trlog(T_v(i).R'*T_vc(i).R);
    ang_vc = [ang_vc, theta];
end

figure;
plot(tt, ang_v,'LineWidth',1.0);
hold on;
plot(ttc, ang_vc,'LineWidth',1.0);
ylabel('theta error');
legend('err_{hardware}', 'err_{command}');

grid on
xlabel('step');

%% 画出机器人跟踪轨迹的速度加速度曲线
delta_T = 0.03;

% 计算角速度和角加速度
angvecp_v = angvp;
angaccp_v = diff(angvecp_v)/delta_T;
anjerkp_v = diff(angaccp_v)/delta_T;

figure;
yyaxis left;
hold on;
plot(ttp(1:end), angvecp_v,'LineWidth',1.0);
ylabel('angle of velocity');

yyaxis right;
hold on
plot(ttp(2:end), angaccp_v,'LineWidth',1.0);
ylabel('angle of acceleration');
legend('ang_{vel}', 'ang_{acc}');
grid on
xlabel('step');

figure;
hold on;
plot(ttp(3:end), anjerkp_v,'LineWidth',1.0);
ylabel('angle of jerk');
legend('ang_{jerk}');
grid on
xlabel('step');

%% 画出下发轨迹的速度加速度曲线

% 计算角速度和角加速度
angvecc_v = [];

delta_T = 0.03;
% 计算姿态角误差
for i = 1:1:(size(T_vc,2) - 1)
    [thetad, w] = trlog(T_vc(i + 1).R'* T_vc(i).R);
    angvecc_v = [angvecc_v, thetad/delta_T];
end

angaccc_v = diff(angvecc_v)/delta_T;
anjerkc_v = diff(angaccc_v)/delta_T;

% 距离 xx 的偏差值

% 位置，速度
figure;
yyaxis left;
hold on;
plot(ttc(2:end), angvecc_v,'LineWidth',1.0);
ylabel('angle of velocity');

yyaxis right;
hold on
plot(ttc(3:end), angaccc_v,'LineWidth',1.0);
ylabel('angle of acceleration');
legend('ang_{vel}', 'ang_{acc}');
grid on
xlabel('step');

% 加速度，加加速度
figure;
hold on;
plot(ttc(4:end), anjerkc_v,'LineWidth',1.0);
ylabel('angle of jerk');
legend('ang_{jerk}');
grid on
xlabel('step');