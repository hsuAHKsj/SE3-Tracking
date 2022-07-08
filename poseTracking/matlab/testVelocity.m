clc;
close all;
clear all;

%% 获取时间轨迹
timePath = "../data/pcs_time.csv";
tt = load(timePath);

%% 获取采集的速度信息
timePath = "../data/test_velocity.csv";
vels = load(timePath);
vel_method1 = vels(:,1);
vel_method2 = vels(:,2);

%% 读取下发的姿态数据
path = "../data/pcs_execute.csv";

data = load(path);
data(:,4:6) = data(:,4:6)*pi/180;

T_vp = [];
for i = 1:1:size(data,1)
   T = SE3(poseToMatrix(data(i,:)));
   T_vp = [T_vp, T];
end

%% 计算旋转速度 

delta_T = 0.025;
angvec_v =[];
% 计算姿态角误差
for i = 1:1:(size(tt) - 1)
    [thetad, w] = trlog(T_vp(i + 1).R'* T_vp(i).R);
    angvec_v = [angvec_v, thetad/(tt(i+1) - tt(i))];
end

% 绘图
figure;
plot(tt(2:end), angvec_v,'LineWidth',1.0);
ylabel('roboVel');
hold on
% 接口1 获得的速度
plot(tt, vel_method1,'LineWidth',1.0);
% 接口2 获得的速度
plot(tt, vel_method2,'LineWidth',1.0);
legend('Diff', 'Vel_{TCP}',  'Jac*q');

grid on
xlabel('second');


function T=poseToMatrix(x)
% X-Y-Z Euler angles模式

% % 将位姿转为末端的齐次变换矩阵 
    gama=x(4);  beta=x(5);  alpha=x(6);
%     gama=x(6);  beta=x(5);  alpha=x(4);
    R=[cos(alpha)*cos(beta) cos(alpha)*sin(beta)*sin(gama)-sin(alpha)*cos(gama) cos(alpha)*sin(beta)*cos(gama)+sin(alpha)*sin(gama);
        sin(alpha)*cos(beta) sin(alpha)*sin(beta)*sin(gama)+cos(alpha)*cos(gama) sin(alpha)*sin(beta)*cos(gama)-cos(alpha)*sin(gama);
        -sin(beta) cos(beta)*sin(gama) cos(beta)*cos(gama)];
    
    T=[R x(1:3)';    0 0 0 1];
 end