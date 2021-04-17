% System Time configuration
sim_time = 1;             % Simulation Time
dt       = 0.001;         % Time Step
num      = sim_time/dt;   % number of steps 
t        = [0:dt:(sim_time-dt)]; % Time array
g = 9.81;

% Electrical Response of Motor
v        = zeros(num);    % Voltage Array
I        = zeros(num);    % Current Array

% Beam Response Characteristics
theta    = zeros(num);    % Beam Angle
theta_d  = zeros(num);    % Beam Angular Velocity
theta_dd = zeros(num);    % Beam Acceleration

% Ball Response Characteristics
x        = zeros(num);    % Ball Position
x_d      = zeros(num);    % Ball Velocity
x_dd     = zeros(num);    % Ball Acceleration

% System Torques
T_b      = zeros(num);    % Torque from ball
T_m      = zeros(num);    % Torque from motor

% Motor Characteristics
R        = 1;            % Resistance
k_t      = 0.36121;      % Torque Constant
k_e      = 0.36121;      % Electrical Constant
L        = 850e-6;       % Motor Inductance
G        = 19.37;        % Gear ratio

% System Interias
J_sys    = zeros(num);    % Load Interia (dynamic)
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
J_b = zeros(num);


% Initial State
x(1) = -85e-3;           % The current position on the plane (starting)
theta(1) = 25;          % degrees
J_b(1) = J_ball(x(1));   % Initial Intertia From Ball

J_sys(1) = J_winding + J_shaft + J_platform + J_b(1);

% Torques of the system
T_b(1) = T_ball(x(1), theta(1));
T_m(1) = T_b(1) + J_sys(1)*theta_dd(1);



for i = 1:num
    v(i) = 0;
    I(i) = (v(i) - k_t*theta_d(i))/R;
    
    T_m(i) = k_t * I(i);
    T_b(i) = T_ball(x(i), theta(i));
    J_b(i+1) = J_ball(x(i)); 
    J_sys(i+1) = J_winding + J_shaft + J_platform + J_b(i);
    
    theta_dd(i+1) = (T_m(i) - T_b(i))/J_sys(i); % TODO: assess with friction
    theta_d(i+1) = theta_d(i) + theta_dd(i)*dt;
    %fprintf('theta_D-%d: %d \n',i, theta_d(i));
    theta(i+1) = theta(i) + theta_d(i)*dt;
    %fprintf('theta-%d: %d \n',i, theta(i));
    
    if (abs(x(i)) < l_platform/2)
        x_dd(i+1) = (5/3)*theta(i)*g;
    end
    x_d(i+1) = x_d(i) + x_dd(i)*dt;
    x(i+1) = x(i) + x_d(i)*dt;
    if (x(i+1) > l_platform/2)
        x(i+1) = l_platform/2;
    end
    if (x(i+1) < - l_platform/2)
        x(i+1) = - l_platform/2;
    end
end

rows = 4;
cols = 2;

% 
subplot(rows,cols,1)
plot(t,v)
ylabel('Voltage [V]')
xlabel('Time [s]')
ylim([-85e-3, 85e-3])

subplot(rows,cols,2)
plot(t,I)
ylabel('Current [V]')
xlabel('Time [s]')

subplot(rows,cols,3)
plot(t,theta)
ylabel('\theta [rad]')
xlabel('Time [s]')

subplot(rows,cols,4)
plot(t,theta_d)
ylabel('\omega [rad/s]')
xlabel('Time [s]')

% ----------
subplot(rows,cols,5)
plot(t,x)
ylabel('x [m]')
xlabel('Time [s]')

subplot(rows,cols,6)
plot(t,x_d)
lab = ylabel('${v} [m/s]$');
xlabel('Time [s]')
set(lab,'Interpreter', 'latex');

subplot(rows,cols,[7, 8])
plot(t,x_dd)
lab = ylabel('${a} [m/s^2]$');
xlabel('Time [s]')
set(lab,'Interpreter', 'latex');
%plot(t,theta_dd)
%lab = ylabel('${\alpha} [rad/s^2]$');
%xlabel('Time [s]')
%set(lab,'Interpreter', 'latex');
