clc;
close all;
clear all;

%% 获取时间轨迹
timePath = "../data/time.csv";

tt = load(timePath);


%% 读取指令数据
path = "../data/cmd.csv";

data = load(path);

q1 = data(:,1);
q2 = data(:,2);
q3 = data(:,3);
q4 = data(:,4);
q5 = data(:,5);
q6 = data(:,6);

%% 读取跟踪数据
pathE = "../data/execute.csv";

dataE = load(pathE);

q1e = dataE(:,1);
q2e = dataE(:,2);
q3e = dataE(:,3);
q4e = dataE(:,4);
q5e = dataE(:,5);
q6e = dataE(:,6);

% 绘图程序
figure;
plot(tt, q5,'LineWidth',1.0);
ylabel('q5');
hold on
plot(tt, q5e,'LineWidth',1.0);
legend('q5_{cmd}', 'q5_{exe}');

grid on
xlabel('second');

% 求速度加速度


