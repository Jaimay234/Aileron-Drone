clear; clc; close all;

% Equation for Roll Dynamics p_dot = (1/Ix)(Lp*p + Lda*da)

% Test Aircraft Roll Parameters

Ix = 0.15; % x-axis moment of inertia. 

% --------- Moment of Inertia Estimated Values --------- %
    % Ix = 0.15 for relatively centralized mass
    % Ix = 0.25 for intermediate distribution
    % Ix = 0.35 for broad mass distribution
    % Ix = 0.44 for uniform rectangular model
% ------------------------------------------------------ %

Lp = -0.706605741; % Roll damping derivative from Aileron.xlsx 
Lda = 0.389; % Aileron control power estimate. From Aileron.xlsx it can be either ~0.343 or ~0.389 depending on fraction of wing span

tspan = [0 10];
t_plot = linspace(tspan(1), tspan(2), 1000); 

y0 = [0; 0]; %Initial Conditions of DE: roll rate p (rad/s), roll angle phi (rad)

wing_sweep_start = 0;
wing_sweep_end = 45;
da_input = 40;

global da_function wing_sweep_func;
da_function = @(t) (t>=0.5 & t<=1.5) * deg2rad(da_input) + (t>=2.5 & t<=3.5)*deg2rad(-da_input) + (t>=5 & t<=6) * deg2rad(da_input) + (t>=7 & t<=8)*deg2rad(-da_input) ; %function that changes input from joystick
wing_sweep_func = @(t) (t<4)*wing_sweep_start + (t>=4 & t<5)*(wing_sweep_end*(t-4)/(5-4)) + (t>=5)*wing_sweep_end; % function that changes wing sweep angle

da_deg = arrayfun(@(t) rad2deg(da_function(t)), t_plot); %converting input function to an array to easily plot
sweep_deg = arrayfun(@(t) wing_sweep_func(t), t_plot); %converting wing sweep function to an array to plot

[t, y] = ode45(@(t, y) roll_eom(t, y, Ix, Lp, Lda), tspan, y0); %solve ode of roll dynamics

p = y(:, 1);
phi = rad2deg(y(:, 2));

figure;
subplot(4,1,1);
plot(t, rad2deg(p), 'LineWidth',2);
xlabel('Time (s)'); ylabel('Roll Rate p (deg/s)');
grid on; title('Aircraft Roll Dynamics Simulation');

subplot(4, 1, 2);
plot(t, phi, 'r', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Roll Angle \phi [deg]');
grid on;

subplot(4, 1, 3);
plot(t_plot, da_deg, 'LineWidth', 2);
xlabel('Time (s)'); ylabel('input \delta _a (deg)');
grid on;

subplot(4, 1, 4);
plot(t_plot, sweep_deg, 'LineWidth', 2);
xlabel('Time (s)'); ylabel('wing sweep \Lambda (deg)');
grid on;

function dydt = roll_eom(t, y, Ix, Lp, Lda)
global da_function wing_sweep_func;

p = y(1);       % State 1: roll rate
phi = y(2);     % State 2: roll angle

delta_a = da_function(t); % Current aileron input
Lambda = deg2rad(wing_sweep_func(t));

% 1. Inertia shrinks as mass moves inward
Ix1 = Ix*cos(Lambda);

% 2. Damping drops due to loss of effective span
Lp_eff = Lp*cos(Lambda);

% 3. Control authority degrades aggressively at high sweep angles
Lda_eff = Lda*(cos(Lambda)^2);

% Combined geometric bank angle penalty from previous step
Lda_eff = Lda_eff*cos(phi);
Lp_eff = Lp_eff*cos(phi);

% Compute derivatives
p_dot = (Lp_eff * p + Lda_eff * delta_a) / Ix1;
phi_dot = p;

dydt = [p_dot; phi_dot];
end