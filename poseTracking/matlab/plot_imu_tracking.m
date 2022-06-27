clc;
close all;
clear all;

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

% 遍历每一个数据构造SO3
figure;
hold on;
axis equal;
T_v = [];
for i = 1:1:size(x,1)
   % 构造SE3 
   t = [x(i), y(i), z(i)];
   quat = quaternion(qw(i), qx(i), qy(i), qz(i));
   SO3 = SO3.convert(rotmat(quat,'point'));
   T = SE3(SO3.R, t);
   T_v = [T_v, T];

   plotframe(T,0.05);
end

%% 获取跟踪实时数据绘制

path = "../data/SE3_P_control_path.csv";

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
T_v = [];
for i = 1:1:size(xp,1)
   % 构造SE3 
   t = [xp(i), yp(i), zp(i)];
   quat = quaternion(qwp(i), qxp(i), qyp(i), qzp(i));
   SO3 = SO3.convert(rotmat(quat,'point'));
   T = SE3(SO3.R, t);
   T_v = [T_v, T];

   plotframe(T,0.05);
end

% 绘制误差曲线图

