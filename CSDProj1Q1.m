%% Control System Design - Project 1 Question 1
%% Parameters:
clear, close all
% parameters:
w_ap = 3.2; zeta_ap = 0.75; % ap - auto pilot
Ka = 4.3; % drag coefiecint
Vw = 5; % [m/s] % wind speed
m = 1230; % [m] % mass

% parameters for SIMULINK:
C_V_tilde = 1;
k_Theta = 0.05;
k_V_tilde = 0.02;
V_nom = 40 * (1000/3600);

%% Section 2 - Simulation
% I.C
V0 = 40 * (1000/3600);

% impulse genreate:
a_yr1_amp = 3;
a_yr1_time = 40;
a_yr2_amp = 3;
a_yr2_time = 70;

%simulation and plots
simulation_output=sim("CSDProj1Q1.slx",'StartTime','0','StopTime','120');
F_th = simulation_output.F_th(:,2);
Theta = simulation_output.Theta(:,2);
V = simulation_output.V(:,2);
X = simulation_output.X(:,2);
Y = simulation_output.Y(:,2);
a_yr = simulation_output.a_yr(:,2);
t = simulation_output.tout;
myplot(t,V,'t [s]','V [m/s]',0,'-')
myplot(t,Theta,'t [s]','$\Theta  [rad]$',0,'-')
myplot(X,Y,'x [m]','y [m]',1,'-')

%% Section 3 - Linearization Assumptions
% impulse resopnse generate
a_yr1_amp = 3;
a_yr1_time = 0;
a_yr2_amp = 3;
a_yr2_time = 0.01;

% simulation and plots
simulation_output=sim("CSDProj1Q1.slx",'StartTime','0','StopTime','50');
Theta = simulation_output.Theta(:,2);
V = simulation_output.V(:,2);
t = simulation_output.tout;
myplot(t,V,'t [s]','V [m/s]',0,'-')
myplot(t,Theta,'t [s]','$\Theta  [rad]$',0,'-')

%% Section 4 -  Controllers Design
% Theta Loop:
P_ap = tf(w_ap^2,[1 2*zeta_ap*w_ap w_ap^2]);
Integrator = tf(1, [1 0]);
P_Thete_tilde = P_ap * Integrator * (1/ V_nom)
k_Theta = 1.5;
C_Theta = k_Theta * V_nom;
L_Theta = C_Theta * P_Thete_tilde
margin(L_Theta)

% V_tilde Loop:
P_V_tilde = tf(1/m , [1 2*Ka*V_nom/m])
k_V_tilde = 10;
L_V_tilde = P_V_tilde * tf(k_V_tilde, [1 0])
margin(L_V_tilde)

%% Section 5 - Linear Simulation
simulation_output=sim("CSDProj1Q1.slx",'StartTime','0','StopTime','250');
V_lin = simulation_output.V_lin(:,2);
Theta_lin = simulation_output.Theta_lin(:,2);
F_th_lin = simulation_output.F_th_lin(:,2);
a_yr_lin = simulation_output.a_yr_lin(:,2);
t_lin = simulation_output.tout;

myplot(t_lin,V_lin,'t [s]','$V [\frac{km}{hour}]$',0,'-')
myplot(t_lin,rad2deg(Theta_lin),'t [s]','$\Theta   [\circ]$',0,'-')
myplot(t_lin,a_yr_lin,'t [s]','$a_{yr}  [\frac{m}{s^2}]$',0,'-')
myplot(t_lin,F_th_lin,'t [s]','$F_{th}  [N]$',0,'-')

%% Section 6 - Controlled Verification Model
simulation_output=sim("CSDProj1Q1.slx",'StartTime','0','StopTime','250');
F_th = simulation_output.F_th_controlled(:,2);
Theta_controlled = simulation_output.Theta_controlled(:,2);
V_controlled = simulation_output.V_controlled(:,2);
X = simulation_output.X_controlled(:,2);
Y = simulation_output.Y_controlled(:,2);
a_yr = simulation_output.a_yr_controlled(:,2);
t = simulation_output.tout;

% graphs:
% Theta:
clf
hold on
myplot(t,rad2deg(Theta_controlled),'t [s]','$\Theta  [\circ]$',0,'-')
myplot(t_lin,rad2deg(Theta_lin),'t [s]','$\Theta   [\circ]$',0,'--')
xlim([40 60])
legend 'Verification Model' 'Linearized Model'
hold off
% V:
figure, clf
hold on
myplot(t,V_controlled,'t [s]','',0,'-')
myplot(t_lin,V_lin,'t [s]','$V [\frac{km}{hour}]$',0,'--')
legend 'Verification Model' 'Linearized Model'
hold off

% displacment:
myplot(X,Y,'x [m]','y [m]',1,'-')

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
