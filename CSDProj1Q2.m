%% Conrol System Design - Project 1 Question 2
% Elevator Problem, Using an hydraulic actuator.
%% Parameters
clear, close all
% parameters:
m = 100; %[kg]
A = 5e-3; %[m^2]
V = 11e-3; %[m^3]
ps = 7e6; %[Pa]
cf = 9e3; %[Ns/m]
ct = 2e-10; %[~SI]
KQ = 2e-3; %[~SI]
rho = 850; %[kg/m^3]
beta = 1e9; %[Pa]
omega_v = 20 * 2*pi; %[rad/sec]
ksi_v = 0.75; %[-]
mL = 1000; %[kg]
Qm = 100; %[liter/min]
Xv_max = 10e-3; %[m]
g = 9.81; % [m/s^2]
%% Section 1 -  Equlibrium Values and Non-Linear Simulation
% state vector [x1 x2 x3] = [x x_dot p_l]
% input vector [u1 u2] = [F_l x_v]

% Equalibrium values:
x_3_star = m * g / A;
u_1_star = ct * m * g / (KQ * A * sqrt((ps - m * g * sign(1)) / rho));
u_2_star = m * g;

% simulate of 1 [t] disturbance
output = sim('CSDProj1Q2.slx','StartTime','0','StopTime','5');
x1 = output.x_1(:,2);
t = output.tout;
myplot(t,x1, 'Time [s]', 'x [m]',0,'-')

%% Section 2 - Linearization and Linear Simulation
syms X_1 X_2 X_3 U_1 U_2 % cf m A ct beta V KQ rho ps
X = [X_1;X_2;X_3];
U =[U_1; U_2];
% state equations:
x_1_dot = X_2;
x_2_dot = A/m * X_3 -cf/m * X_2 - U_2 / m;
x_3_dot = - (A * beta / V) * X_2 - (ct * beta / V) * X_3 + (KQ * beta / V) * U_1 * sqrt((ps-X_3)/rho) ;
F = [x_1_dot;x_2_dot;x_3_dot];
G = X_1;
% linearized state space:
A_sys = double(subs(jacobian(F,X),[X_3 U_1],[x_3_star u_1_star]));
B_sys = double(subs(jacobian(F,U),[X_3 U_1],[x_3_star u_1_star]));
C_sys = double(jacobian(G,X));
D_sys = double(jacobian(G,U));

sys = ss(A_sys,B_sys,C_sys,D_sys);
t_lin = linspace(0,20,2000);
u = double([u_1_star*ones(size(t_lin)) ; ...
    1000 * g * heaviside(t_lin) - 1000 * g * heaviside(t_lin-1) + u_2_star]);

% linear simulation:
y = lsim(sys,u,t_lin);
% plots:
figure, clf
hold on
myplot(t_lin,y,'Time [s]', 'x [m]',0,'-')
myplot(t,x1, 'Time [s]', 'x [m]',0,'--')
legend 'Linearized Model' 'Non-Linear Model'
hold off
%clearvars F G y u t_lin X U X_1 X_2 X_3 U_1 U_2 x_1_dot x_2_dot x_3_dot

%% Section 3 - Controller Design
s = tf('s');
[b,a] = ss2tf(A_sys,B_sys,C_sys,D_sys,1);
G_xv_x = tf(b,a)
G_u_xv = tf(omega_v^2,[1 2*ksi_v*omega_v omega_v^2])
G_u_x = zpk(G_u_xv * G_xv_x)
[b,a] = ss2tf(A_sys,B_sys,C_sys,D_sys,2);
G_Fl_x = tf(b,a);
margin(G_u_x)
c = 0.6*(10*s+20)/(10*s);
L = c*G_u_x;
margin(L)
stepinfo(feedback(L,1))

%% Section 4 - Simulation Verification Model Vs. Linearized Model
output = sim("CSDProj1Q2.slx",'StartTime','0','StopTime','5');
x_1_lin = output.x_1_lin(:,2);
x_1_controlled = output.x_1_ver_controlled(:,2);
Q_controlled = output.Q_controlled(:,2);
x_v_controlled = output.u_1_controlled(:,2);
t = output.tout;

% plots:
figure,clf
hold on
myplot(t,x_1_controlled, 'Time [s]', 'x [m]',0,'-')
myplot(t,x_1_lin, 'Time [s]', 'x [m]',0,'--')
legend 'Verification Model' 'Linearized Model'
hold off
figure
myplot(t,Q_controlled, 'Time [s]', '$Q   [\frac{ltr}{min}]$',0,'-')
figure
myplot(t,x_v_controlled, 'Time [s]', '$x_v [m]$',0,'-')

%% Section 5 - Ramp Reference Simulation
output = sim("CSDProj1Q2.slx",'StartTime','0','StopTime','15');
x_1_controlled = output.x_1_ver_controlled(:,2);
Q_controlled = output.Q_controlled(:,2);
x_v_controlled = output.u_1_controlled(:,2);
t = output.tout;
x_1_ref = output.x_1_ref(:,2);
% plots:
figure, clf
hold on
myplot(t,x_1_ref, 'Time [s]', '$x_{ref} [m]$',0,'-')
myplot(t,x_1_controlled, 'Time [s]', 'x [m]',0,'-')
legend 'r' 'y'
hold off
figure
myplot(t,Q_controlled, 'Time [s]', '$Q   [\frac{ltr}{min}]$',0,'-')
figure
myplot(t,x_v_controlled, 'Time [s]', '$x_v [m]$',0,'-')

%% Functions:
function myplot(x,y, x_label, y_label,axis_equal,line_style)
    plot(x,y,'LineWidth',2.2,'LineStyle',line_style)
    xlabel(x_label,"Interpreter",'latex',"FontSize",14,"FontWeight","bold")
    ylabel(y_label,"Interpreter","latex","FontSize",14,"FontWeight","bold")
    grid on
    if (axis_equal)
        axis equal
    end
end

