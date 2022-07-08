clc;
close all;
clear all;

%%
path = "../data/se_interpolation.csv";

%file_base = "se2_interp_slerp";
%file_base = "se2_interp_cubic";
file_base = "se2_interp_cnsmooth";

extension = ".csv";

file = fullfile(path, [file_base extension]);

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

%% SE2图片转化
r = 0.1; % magnitude (length) of arrow to plot 画图矢量的长度

x = data(2:end,1);
y = data(2:end,2);
t = data(2:end,3);

for i = 1:size(t,1)
    if t(i) < -1
        t(i) = t(i) + pi*2;
    end
end

td = diff(t);
xd = diff(x);
yd = diff(y);

% 如果 t < 0 

u = r * cos(t); % convert polar (theta,r) to cartesian
v = r * sin(t);

%% 画图
figure;
% quiver(x(1:num_k_pts),y(1:num_k_pts),u(1:num_k_pts),v(1:num_k_pts),'color',[0 0 1]);
hold on

quiver(x(num_k_pts+1:end),y(num_k_pts+1:end),u(num_k_pts+1:end),v(num_k_pts+1:end),'color',[1 0 0]);
set(get(gca, 'title'), 'string', title);
hold off


%% 画出跟踪的轨迹坐标

%%%%%%%%%%%%%%%%%
path = "../data/P_control_path.csv";
data = load(path);

r = 0.1;
xp = data(:,1);
yp = data(:,2);
tp = data(:,3);


for i = 1:size(tp,1)
    if tp(i) < -1
        tp(i) = tp(i) + pi*2;
    end
end


tpd = diff(tp);
xpd = diff(xp);
ypd = diff(yp);

up = r * cos(tp); % convert polar (theta,r) to cartesian
vp = r * sin(tp);

hold on
quiver(xp,yp,up,vp,'color',[0 1 0]);
set(get(gca, 'title'), 'string', title);
hold off

% 计算参数坐标误差
err_x = x(num_k_pts+1:end-1) - xp;
err_y = y(num_k_pts+1:end-1) - yp;
err_t = t(num_k_pts+1:end-1) - tp;

err_xd = xd(num_k_pts+2:end) - xpd;
err_yd = yd(num_k_pts+2:end) - ypd;
err_td = td(num_k_pts+2:end) - tpd;


figure;
yyaxis left;
hold on;
plot(sqrt(err_x.*err_x + err_y .*err_y),'LineWidth',1.0);
% plot(err_y,'LineWidth',1.0);
ylabel('x,y position error');

yyaxis right;
hold on
plot(err_t,'LineWidth',1.0);
ylabel('theta error');
legend('err_{xy}', 'err_t');

grid on
xlabel('step');


figure;
yyaxis left;
hold on;
plot(sqrt(err_xd.*err_xd + err_yd .*err_yd),'LineWidth',1.0);
ylabel('x,y velocity error');

yyaxis right;
hold on
plot(abs(err_td),'LineWidth',1.0);
ylabel('angle velocity error');
legend('errd_{xy}', 'errd_t');

grid on
xlabel('step');

%% 画出轨迹图
figure;
yyaxis left;
hold on;
plot(x(num_k_pts+1:end-1),'LineWidth',1.0);
plot(y(num_k_pts+1:end-1),'LineWidth',1.0);
ylabel('x,y position of Trajetory');

yyaxis right;
hold on
plot(t(num_k_pts+1:end-1),'LineWidth',1.0);
ylabel('theta of Trajectory');
legend('x', 'y', 't');

grid on
xlabel('step');

%% 画出速度加速度曲线图

DelatT = 0.04;
tpd = diff(tp)/DelatT;
tpdd = diff(tpd)/DelatT;
tpdd = [0; tpdd];

figure;
yyaxis left;
hold on;
plot(tpd,'LineWidth',1.0);
ylabel('angle vel of theta');

yyaxis right;
hold on
plot(tpdd,'LineWidth',1.0);
ylabel('angle acc of theta');
legend('td', 'tdd');

grid on
xlabel('step');

return;
