clc;

close all;
clear all;

%% 获取imu实时数据绘制
delta_T = 0.05;

path = "../data/SE3_P_control_imu_uniqueThread.csv";

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

angvec_v = [];
% 计算传感器的速度
for i = 1:1:(size(T_v,2) - 1)
    [thetad, w] = trlog(T_v(i + 1).R'* T_v(i).R);
    angvec_v = [angvec_v, thetad/delta_T];
end

%% 获取实时下发数据
path = "../data/SE3_P_control_conLaw_uniqueThread.csv";

dataTrack = load(path);

xc = dataTrack(:,1);
yc = dataTrack(:,2);
zc = dataTrack(:,3);

qxc = dataTrack(:,4);
qyc = dataTrack(:,5);
qzc = dataTrack(:,6);
qwc = dataTrack(:,7);
ttc = dataTrack(:,8);

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

path = "../data/SE3_P_control_robot_uniqueThread.csv";

dataTrack = load(path);

xp = dataTrack(:,1);
yp = dataTrack(:,2);
zp = dataTrack(:,3);

qxp = dataTrack(:,4);
qyp = dataTrack(:,5);
qzp = dataTrack(:,6);
qwp = dataTrack(:,7);
ttp = dataTrack(:,8);

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

% 传感器速度

%% 画出机器人跟踪轨迹的速度加速度曲线
angvecp_v =[];

% 计算姿态角误差
for i = 1:1:(size(T_vp,2) - 1)
    [thetad, w] = trlog(T_vp(i + 1).R'* T_vp(i).R);
    angvecp_v = [angvecp_v, thetad/(tt(i+1)-tt(i))];
end

% 计算角速度和角加速度
angaccp_v = diff(angvecp_v)/delta_T;
anjerkp_v = diff(angaccp_v)/delta_T;

figure;
yyaxis left;
hold on;
plot(ttp(2:end), angvecp_v,'LineWidth',1.0);
ylabel('angle of velocity');

yyaxis right;
hold on
plot(ttp(3:end), angaccp_v,'LineWidth',1.0);
ylabel('angle of acceleration');
legend('ang_{vel}', 'ang_{acc}');
grid on
xlabel('step');

figure;
hold on;
plot(ttp(4:end), anjerkp_v,'LineWidth',1.0);
ylabel('angle of jerk');
legend('ang_{jerk}');
grid on
xlabel('step');

%% 画出下发轨迹的速度加速度曲线

% 计算角速度和角加速度
angvecc_v = [];

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


%% 计算通讯时间

difftt = diff(tt);


TimePath = "../data/CommunicationTime.csv";
timeInterpert = load(TimePath);

figure;
hold on;
plot(ttc, timeInterpert,'LineWidth',1.0);
plot(ttc(2:end),difftt,'LineWidth',1.0);
ylabel('time(s)');
legend('time_{com}', 'time_{loop}');
grid on
xlabel('step');


%% 比较传感器和机器人的速度
% 获取实时下发数据
path = "../data/angleVel.csv";

angVel = load(path);

figure;
hold on;
plot(tt(1:end), angVel,'LineWidth',1.0);
plot(ttc(2:end), angvecc_v,'LineWidth',1.0);
plot(ttp(2:end), angvecp_v,'LineWidth',1.0);

ylabel('time(s)');
legend('imu_{real}',  'vel_{control}','vel_{robot}');
grid on
xlabel('step');


%% 临时画图 对于直接进行位置差分和IMU原有速度进行比较
% 获取实时下发数据
path = "../data/angleVel_Diff.csv";
angleVel_Diff = load(path);

path = "../data/angleVel.csv";
angVel = load(path);

figure;
hold on;

plot(tt(1:end), angVel,'LineWidth',1.0);
plot(tt(1:end), angleVel_Diff,'LineWidth',1.0);

ylabel('time(s)');
legend('imu_{real}', 'imu_{differ}');
grid on
xlabel('step');
