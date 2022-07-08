clc;
close all;
clear all;

%% 获取imu实时数据绘制
path = "../data/SE3_P_control_imu_bTrans.csv";

dataImu = load(path);

x = dataImu(:,1);
y = dataImu(:,2);
z = dataImu(:,3);

qx = dataImu(:,4);
qy = dataImu(:,5);
qz = dataImu(:,6);
qw = dataImu(:,7);

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

%% 获取跟踪实时数据绘制

path = "../data/SE3_P_control_conLaw_bTrans.csv";

dataTrack = load(path);

xp = dataTrack(:,1);
yp = dataTrack(:,2);
zp = dataTrack(:,3);

qxp = dataTrack(:,4);
qyp = dataTrack(:,5);
qzp = dataTrack(:,6);
qwp = dataTrack(:,7);

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
err_x = x(1:end) - xp;
err_y = y(1:end) - yp;
err_z = z(1:end) - zp;

ang_v = [];
% 计算姿态角误差
for i = 1:1:size(T_vp,2)
    [theta, w] = trlog(T_vp(i).R* T_v(i).R');
    ang_v = [ang_v, theta];
end

figure;
plot(ang_v,'LineWidth',1.0);
ylabel('theta error');
legend('err_{angle}');

grid on
xlabel('step');

%% 画出跟踪轨迹的速度加速度曲线

% 计算角速度和角加速度
angvec_v = [];
angacc_v = []
anjerk_v = [];

delta_T = 0.04;
% 计算姿态角误差
for i = 1:1:(size(T_vp,2) - 1)
    [thetad, w] = trlog(T_vp(i + 1).R'* T_vp(i).R);
    angvec_v = [angvec_v, thetad/delta_T];
end

angacc_v = diff(angvec_v)/delta_T;
anjerk_v = diff(angacc_v)/delta_T;

figure;
yyaxis left;
hold on;
plot(angvec_v,'LineWidth',1.0);
ylabel('angle of velocity');

yyaxis right;
hold on
plot(angacc_v,'LineWidth',1.0);
ylabel('angle of acceleration');
legend('ang_{vel}', 'ang_{acc}');
grid on
xlabel('step');

figure;
hold on;
plot(anjerk_v,'LineWidth',1.0);
ylabel('angle of jerk');
legend('ang_{jerk}');
grid on
xlabel('step');