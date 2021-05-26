% System Time configuration
sim_time = 1.6;             % Simulation Time
dt       = 0.001;         % Time Step
controller_dt = 10;
num      = sim_time/dt;   % number of steps 
t        = [0:dt:(sim_time-dt)]; % Time array
g = 9.81;

% Electrical Response of Motor
v        = zeros(1, num);    % Voltage Array
I        = zeros(1, num);    % Current Array

% Beam Response Characteristics
max_angle = 30*(pi/180);
theta    = zeros(1, num);    % Beam Angle
theta_d  = zeros(1, num);    % Beam Angular Velocity
theta_dd = zeros(1, num);    % Beam Acceleration

% Ball Response Characteristics
x        = zeros(1, num);    % Ball Position
x_d      = zeros(1, num);    % Ball Velocity
x_dd     = zeros(1, num);    % Ball Acceleration
x_q        = zeros(1, num);    % Ball Position
x_d_q      = zeros(1, num);    % Ball Velocity
x_dd_q     = zeros(1, num);    % Ball Acceleration

% System Torques
T_b      = zeros(1, num);    % Torque from ball
T_m      = zeros(1, num);    % Torque from motor

% Motor Characteristics
R        = 1;            % Resistance
k_t      = 0.36121;      % Torque Constant
k_e      = 0.36121;      % Electrical Constant
L        = 850e-6;       % Motor Inductance
G        = 19.37;        % Gear ratio

% System Interias
J_sys    = zeros(1, num);    % Load Interia (dynamic)
J_m      = 2.23e-3;       % Motor Intertia
m_winding  = 90e-3;       % Winding Mass
r_winding  = 11.5e-3;     % Winding Radius
J_winding  = ((m_winding*r_winding^2)/2)*G; % Winding Intertia
m_shaft    = 0.0137;      % Shaft Mass
r_shaft    = 3e-3;        % Shaft Radius
J_shaft    = m_shaft*(r_shaft^2)/2; % Shaft Intertia
m_platform = 120e-3;      % Platform mass
l_platform = 170e-3;      % Platform length
J_platform = m_platform*(l_platform^2)/12; % Plaform Intertia
% The proper J_ball is based on it's location on the plane using
% Parallel axis theorem
J_b = zeros(1, num);


% Initial State
x(1) = -85e-3;           % The current position on the plane (starting)
theta(1) = 30*(pi/180);          % degrees
J_b(1) = J_ball(x(1));   % Initial Intertia From Ball
J_sys(1) = J_winding + J_shaft + J_platform + J_b(1);

% Torques of the system
T_b(1) = T_ball(x(1), theta(1));
T_m(1) = T_b(1) + J_sys(1)*theta_dd(1);

G = 1;
Kp = G*40;  % 8
Ki = G*0;  % 0
Kd = G*4;   % 3
Kdd = G*0.7;  % 2

error = zeros(1, num);
cum_error = zeros(1, num);
rate_error = zeros(1, num);
d_rate_error = zeros(1, num);
response = zeros(1, num);
step_response = zeros(1, num);
kp_response = zeros(1, num);
ki_response = zeros(1, num);
kd_response = zeros(1, num);
kdd_response = zeros(1, num);

error(1) = x(1);
cum_error(1) = error(1)*dt;
rate_error(1) = 0;
d_rate_error(1) = 0;

current_rate = 0;
last_rate = 0;
last_error = 0;
current_error = 0;

for i = 1:num-1
    
    quantise = (mod(i, controller_dt) == 0);
    
    kp_response(i) = Kp*error(i);
    
    if kp_response(i) >= 12
        kp_response(i) = 12;
    end
    if kp_response(i) <= -12
        kp_response(i) = -12;
    end
    ki_response(i) = Ki*cum_error(i);
    if ki_response(i) >= 12
        ki_response(i) = 12;
    end
    if ki_response(i) <= -12
        ki_response(i) = -12;
    end
    kd_response(i) = Kd*rate_error(i);
    if kd_response(i) >= 12
        kd_response(i) = 12;
    end
    if kd_response(i) <= -12
        kd_response(i) = -12;
    end
    kdd_response(i) = Kdd*d_rate_error(i);
    if kdd_response(i) >= 12
        kdd_response(i) = 12;
    end
    if kdd_response(i) <= -12
        kdd_response(i) = -12;
    end
    response(i) = -(Kp*error(i) + Ki*cum_error(i) + Kd*rate_error(i) + Kdd*d_rate_error(i));

    v(i) = response(i);

    %v(i) = 0;
    if (v(i) > 12) %
        v(i) = 12; 
    end
    if (v(i) < -12)
        v(i) = -12;
    end

    I(i) = (v(i) - k_t*theta_d(i))/R; %TODO: assess with inductance
    
    T_m(i) = k_t * I(i);
    T_b(i) = T_ball(x(i), theta(i));
    J_b(i) = J_ball(x(i)); 
    J_sys(i) = J_winding + J_shaft + J_platform + J_b(i);
    
    if (theta(i) >= max_angle && T_m(i) < 0)
        theta_dd(i+1) = 0;
        theta_d(i+1) = 0;
    elseif (theta(i) <= -max_angle && T_m(i) > 0)
        theta_dd(i+1) = 0;
        theta_d(i+1) = 0;
    else
        theta_dd(i+1) = (T_m(i) + T_b(i))/J_sys(i); % TODO: assess with friction
        theta_d(i+1) = theta_d(i) + theta_dd(i)*dt;
    end
   
    theta(i+1) = theta(i) + theta_d(i)*dt;
    
    if (theta(i+1) > max_angle)
        theta(i+1) = max_angle;
    end
    if (theta(i+1) < -max_angle)
        theta(i+1) = -max_angle;
    end
    
    if (x(i) >= l_platform/2 && theta(i) > 0)
        x_dd(i+1) = 0;
        x_d(i+1) = 0;
    elseif (x(i) <= -l_platform/2 && theta(i) < 0)
        x_dd(i+1) = 0;
        x_d(i+1) = 0;
    else
        x_dd(i+1) = (5/7)*theta(i+1)*g;
        x_d(i+1) = x_d(i) + x_dd(i)*dt;
    end
    
    x(i+1) = x(i) + x_d(i)*dt;
    step_response(i+1) = x(1) - x(i);
        
    if (x(i+1) > l_platform/2)
        x(i+1) = l_platform/2;
    end
    if (x(i+1) < - l_platform/2)
        x(i+1) = - l_platform/2;
    end
    
    if(quantise)
        error(i+1) = x(i+1);
        cum_error(i+1) = last_error + error(i+1)*(dt*controller_dt);
        if i > controller_dt
            rate_error(i+1) = (error(i+1)-last_error)/(dt*controller_dt);
        end
        last_error = error(i+1);
        if i > controller_dt
            d_rate_error(i+1) = (rate_error(i+1) - last_rate)/(dt*controller_dt);
        end
        if d_rate_error(i+1) >= 9.8
            d_rate_error(i+1) = 9.8;
        end
        if d_rate_error(i+1) <= -9.8
            d_rate_error(i+1) = -9.8;
        end
        last_rate = rate_error(i+1);
    else
        error(i+1) = error(i);
        cum_error(i+1) = cum_error(i);
        rate_error(i+1) = rate_error(i);
        d_rate_error(i+1) = d_rate_error(i);
    end
end

for i=1:num
    theta(i) = theta(i)*(180/pi);
end

rows = 5;
cols = 2;

subplot(rows,cols,[1,2])
plot(t,v,'DisplayName', 'Total Voltage')
hold on
plot(t,-kp_response,'DisplayName', 'Kp')
plot(t,-ki_response,'DisplayName', 'Ki')
plot(t,-kd_response,'DisplayName', 'Kd')
plot(t,-kdd_response,'DisplayName', 'Kdd')
hold off
title('Voltage Terms')
ylabel('Voltage [V]')
%xlabel('Time [s]')
legend

%subplot(rows,cols,2)
%plot(t,I)
%ylabel('Current [A]')
%xlabel('Time [s]')

subplot(rows,cols,[3,4])
plot(t,theta)
title('Beam Angle')
ylabel('\theta [Degrees]')
%xlabel('Time [s]')

%subplot(rows,cols,4)
%plot(t,theta_d)
%ylabel('\omega [rad/s]')
%xlabel('Time [s]')

% ----------
subplot(rows,cols,[5,6])
plot(t,x_dd,'DisplayName', 'Real Ball Acceleration')
hold on
plot(t,d_rate_error,'DisplayName', 'Controller Ball Acceleration')
hold off
title('Ball Acceleration')
lab = ylabel('${a} [m/s^2]$');
%xlabel('Time [s]')
set(lab,'Interpreter', 'latex');
legend

subplot(rows,cols,[7,8])
plot(t,x_d,'DisplayName', 'Real Ball Velocity')
hold on
plot(t,rate_error,'DisplayName', 'Controller Ball Velocity')
hold off
title('Ball Velocity')
lab = ylabel('${v} [m/s]$');
%xlabel('Time [s]')
set(lab,'Interpreter', 'latex');
legend


subplot(rows,cols,[9, 10])
plot(t,x,'DisplayName', 'Real Ball Position')
hold on
plot(t,error,'DisplayName', 'Controller Ball Position')
hold off
title('Ball Position')
ylabel('x [m]')
xlabel('Time [s]')
%ylim([-85e-3, 85e-3])
legend

%subplot(rows,cols,[9, 10])
figure;
s = stepinfo(step_response, t, 30e-3)
plot(t,step_response)
ylabel('Amplitude')
xlabel('Time [s]')



