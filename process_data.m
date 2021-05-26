[t_t, rot, x_t, p, v, d, a, dd, res] = textread('capture.txt', ...
    't: %dr: %d, x: %f, p: %f, v: %f, d: %f, a: %f, dd: %f, res: %f', 2105);

initial_time = t_t(1);
beam_len = 170;
ball_radius = 13.5;
beam_travel = beam_len - 2*ball_radius;
ball_start = 24;
ball_end = 988;
real_travel = ball_end - ball_start;
ball_center = 512;
dist_ratio = beam_travel/real_travel;

step_mid = 16.317
fin_time = 11
start_time = 1.25

t_tmp = [];
idx = 1;
start_count = 0;
end_count = 0;
for i=1:length(t_t)
    if ((t_t(i)<(initial_time+(fin_time*1000))) && (t_t(i)>(initial_time+(start_time*1000))))
        disp(t_t(i))
        if (start_count == 0)
            start_count = i;
        end
        t_tmp(idx) = ((t_t(i) - initial_time)/1000) - start_time;
        idx = idx + 1;
    end
end

num = length(t_tmp) - start_count;

% Processed Data
t = zeros(1, num);
for i=1:num
    t(i) = t_tmp(i);
end
theta = zeros(1, num);    % Ball Position
x = zeros(1, num);    % Ball Position
p_r = zeros(1, num);
x_d = zeros(1, num);    % Ball Velocity
v_r = zeros(1, num);
x_dd = zeros(1, num);    % Ball Acceleration
a_r = zeros(1, num);
res_r = zeros(1,num);

idx = 1;
for i=start_count:length(t_tmp)-1
    x(idx) = x_t(i)*dist_ratio;
    p_r(idx) = p(i);
    x_d(idx) = v(i);
    v_r(idx) = d(i);
    x_dd(idx) = a(i);
    a_r(idx) = dd(i);
    res_r(idx) = res(i);
    theta(idx) = rot(i);
    idx = idx+1;
end



% ----------------- PLOTS
rows = 5;
cols = 2;

subplot(rows,cols,[1,2])
plot(t,res_r,'DisplayName', 'Total Voltage')
hold on
plot(t,p_r,'DisplayName', 'Kp')
plot(t,v_r,'DisplayName', 'Kd')
plot(t,a_r,'DisplayName', 'Kdd')
hold off
title('Voltage Terms')
ylabel('Voltage [V]')
%xlabel('Time [s]')
legend


subplot(rows,cols,[3,4])
plot(t,theta)
title('Beam Angle')
ylabel('\theta [Degrees]')
%xlabel('Time [s]')


% ----------
subplot(rows,cols,[5,6])
plot(t,x_dd,'DisplayName', 'Ball Acceleration')
title('Ball Acceleration')
lab = ylabel('${a} [m/s^2]$');
%xlabel('Time [s]')
set(lab,'Interpreter', 'latex');
legend

subplot(rows,cols,[7,8])
plot(t,x_d,'DisplayName', 'Ball Velocity')
title('Ball Velocity')
lab = ylabel('${v} [m/s]$');
%xlabel('Time [s]')
set(lab,'Interpreter', 'latex');
legend


subplot(rows,cols,[9, 10])
plot(t,x,'DisplayName', 'Ball Position')
title('Ball Position')
ylabel('x [m]')
xlabel('Time [s]')
legend

%subplot(rows,cols,[9, 10])
figure;
s = stepinfo(x, t, 16.317)
plot(t,x)
ylabel('mm')
xlabel('Time [s]')