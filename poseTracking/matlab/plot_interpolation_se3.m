clc;
close all;
clear all;

%%
path = "../data/se3_interpolation.csv";

data = load(path);

'num k points'
num_k_pts = data(1,1)

'Interpolation method'
interp_method = data(1,2)

switch interp_method
  case 0
    method = 'SLERP'
  case 1
    method = 'CUBIC'
  case 2
    method = 'CN smooth'
  otherwise
    method = 'Unknown'
end

title = strcat('SE2 Interpolation (', method, ')');

'num interpolated points'
total_pts = size(data(2+num_k_pts:end,1), 1)

%% SE3图片转化,并绘制矩阵
x = data(num_k_pts+2:end,1);
y = data(num_k_pts+2:end,2);
z = data(num_k_pts+2:end,3);

qx = data(num_k_pts+2:end,4);
qy = data(num_k_pts+2:end,5);
qz = data(num_k_pts+2:end,6);
qw = data(num_k_pts+2:end,7);

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

%% 读取并比较误差

path = "../data/SE3_PP_control_path.csv";

data = load(path);

xp = data(1:end,1);
yp = data(1:end,2);
zp = data(1:end,3);

qxp = data(1:end,4);
qyp = data(1:end,5);
qzp = data(1:end,6);
qwp = data(1:end,7);

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

%% 误差曲线图，

% 计算位置误差， 计算姿态角度误差

% 计算参数坐标误差
err_x = x(1:end-1) - xp;
err_y = y(1:end-1) - yp;
err_z = z(1:end-1) - zp;

ang_v = [];
% 计算姿态角误差
for i = 1:1:size(T_vp,2)
    [theta, w] = trlog(T_vp(i).R'* T_v(i).R);
    ang_v = [ang_v, theta];
end

figure;
yyaxis left;
hold on;
plot(sqrt(err_x.*err_x + err_y .*err_y + err_z .*err_z),'LineWidth',1.0);
% plot(err_y,'LineWidth',1.0);
ylabel('position error');

yyaxis right;
hold on
plot(ang_v,'LineWidth',1.0);
ylabel('theta error');
legend('err_{xyz}', 'err_{angle}');

grid on
xlabel('step');


%% 旋转速度，加速度曲线图

% 计算角速度和角加速度
angvec_v = [];
delta_T = 0.04;
% 计算姿态角误差
for i = 5:1:size(T_vp,2) - 1
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

%% 系统指令


return;
