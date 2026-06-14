m=1200; C_d=0.32; A_f=2.2; rho=1.225; C_rr=0.015;
R_w=0.30; G=3.5; J_e=0.5; J_w=1.2; T_max=250; g=9.81;
Tsc = 5e-5;
Ts = 5e-6;

t2    = [0  10  10  13  13  16  16  20]';
grade = [0   0   5   5  -3  -3   0   0]';
Road_grade = timeseries(grade, t2);

a_cg = 1.1;
b_cg = 1.5;
h_cg = 0.55;

% --- Brake demand (0 to 1) ---
t_brake = [0  13   13   16   16   19   19   20]';
brake   = [0  0   0.3  0.3  0.85  0.85   0    0]';
brake_demand = timeseries(brake, t_brake);

% --- Throttle command (0 to 1) ---
t_throttle = [0    8   10   13   16   19   20]';
throttle   = [0    0.3979   0.3647   0.2984   0.1990    0    0]';
throttle_cmd = timeseries(throttle, t_throttle);

% --- Fault Injection Triggers (1-second pulses, staggered) ---
% Triggering Open Circuit for 1 second (t=3s to t=4s)
t_fault_oc   = [0    2.999   3.000   4.000   4.001   20]';
fault_sig_oc = [0    0       0       0       0       0]';
trigger_oc = timeseries(fault_sig_oc, t_fault_oc);

% Triggering Short Circuit for 1 second (t=6s to t=7s)
t_fault_sc   = [0    5.999   6.000   7.000   7.001   20]';
fault_sig_sc = [1    1       1       1      1       1]';
trigger_sc = timeseries(fault_sig_sc, t_fault_sc); 

% --- Thermal Fault Injection ---
% Injecting 1 Megawatt of heat for 4 seconds (t=10s to t=14s)
t_fault_th   = [0   0.999 1.000   2.000  2.001 9.999   10.000   14.000   14.001   20]';
fault_sig_th = [0   0      0     0    0      0       0      0      0        0]';
trigger_th = timeseries(fault_sig_th, t_fault_th);

% --- Reset Command Trigger ---
% Firing a 0.1-second reset pulse shortly after each electrical fault clears.
% Pulse 1: 4.1s to 4.2s (resets the OC fault latch)
% Pulse 2: 7.1s to 7.2s (resets the SC fault latch)
t_reset   = [0    4.099  4.100  4.200  4.201  7.099  7.100  7.200  7.201  20]';
reset_sig = [0    0      1      1      0      0      1      1      0      0]';
reset_cmd = timeseries(reset_sig, t_reset);
%% --- Wind velocity (w in m/s, positive = headwind, negative = tailwind) ---
% Profile: Calm start → building headwind during cruise → 
%          crosswind gust during braking → calm at stop
% Realistic urban/highway wind: 0-15 m/s

%% --- Wind velocity (m/s, positive = headwind, negative = tailwind) ---
% Beaufort scale reference:
% 2 m/s = light breeze, 5 m/s = gentle breeze, 8 m/s = fresh breeze
% 12 m/s = strong breeze — this is quite harsh, reduces it

t_wind = [0;    3;    6;    8;    10;   13;   15;   18;   20];
w_wind = [1.5;  1.5;  4;    5;    5;    3;    -2;   -2;   0];
% t=0-3s:   calm start, 1.5 m/s ambient
% t=3-6s:   light headwind builds as vehicle picks up speed
% t=6-8s:   moderate headwind 4 m/s on open road
% t=8-10s:  steady 5 m/s headwind during cruise (~18 km/h wind)
% t=10-13s: wind easing as vehicle decelerates
% t=13-15s: shifting to mild tailwind
% t=15-18s: gentle tailwind -2 m/s during braking
% t=18-20s: wind settles to zero at stop

wind_velocity = timeseries(w_wind, t_wind);

