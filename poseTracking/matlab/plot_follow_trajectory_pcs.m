clc;
close all;
clear all;

%% 获取时间轨迹
timePath = "../data/pcs_time.csv";
tt = load(timePath);

%% 读取下发的姿态数据
path = "../data/pcs_cmd.csv";

data = load(path);
pcs1 = data(:,1);
pcs2 = data(:,2);
pcs3 = data(:,3);
pcs4 = data(:,4);
pcs5 = data(:,5);
pcs6 = data(:,6);

%% 读取跟踪数据
pathE = "../data/pcs_execute.csv";

dataE = load(pathE);

pcs1e = dataE(:,1);
pcs2e = dataE(:,2);
pcs3e = dataE(:,3);
pcs4e = dataE(:,4);
pcs5e = dataE(:,5);
pcs6e = dataE(:,6);

% 绘图程序
figure;
plot(tt, pcs2,'LineWidth',1.0);
ylabel('pcs_{2}');
hold on
plot(tt, pcs2e,'LineWidth',1.0);
legend('pcs2_{cmd}', 'pcs2_{exe}');

grid on
xlabel('second');

% 求速度加速度
