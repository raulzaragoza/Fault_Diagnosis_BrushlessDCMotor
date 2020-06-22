clear all
close all
clc
%% GENERAL PARAMETERS
J = 50*10^(-3)*10^(-4); %kg.m^2
b=3.2*10^(-3);%Facteur couple/vitesse - impédance nulle (mNm/ (rad/s)
tau_m = J/b;
Ke = 0.0745; % V/rad/sec
Kt=74.5*10^(-3);%0.01 N.m/Amp
R = 2*1.72; %Ohm
L = 2*3.8*10^(-3); %H
tau_e=L/R;
num=Kt;
den=[J*L, J*R+b*L, Ke*Kt+b*R];
winf=24*Kt/(Ke*Kt+b*R);
rpm_inf=60*winf/(2*pi);
P_V_encoder=1024; %Pulsos per volta encoder
Tsd=0.0001;
Tsc=0.0001;
T_sim=100;
%% NO FAULT parameters initialization
t_fault=10;
noise_amp=0.07;
t_pred=inf;
abrup_incip=1; % 0 per abruptes, 1 per incipients
% R fault
R_fault=0;
ramp_f_R=0;
vel_R=0.001;
R_nonlinear=1;
tol_R=0.05;
% J fault
J_fault=0;
ramp_f_J=0;
vel_J=0.5;
J_nonlinear=1;
tol_J=0.05;
% STUCK POSITION fault
pos_stuck=0;
stuck_duration=1;
% CURRENT SENSOR fault
f_i=0;
vel_i=0;
ramp_i=0;
% L fault
ramp_f_L=0;
L_fault=0;
vel_L=0.1;
L_nonlinear=1;
tol_L=0.05;
% B fault
ramp_f_B=0;
B_fault=0;
vel_B=0.001;
B_nonlinear=1;
tol_B=0.05;
% Vdc fault
ramp_f_Vdc=0;
Vdc_fault=0;
vel_Vdc=0.01;
% Load fault
ramp_f_Load=0;
vel_Load=0.01;
Load_fault=0;
delta_load=0.01;

%% MOTOR SYSTEMS
% ELECTRIC system
sys_el_idc=tf(1,[L R]); 
sys_el_id=c2d(sys_el_idc,Tsd,'zoh');
% MECHANIC system
tfw=tf(1,[J b]);
sys_mec_wd=c2d(tfw,Tsd,'zoh');
% RUN SYSTEM
open_system('Simple_model_close_loop_vel_control_9');
aux=sim('Simple_model_close_loop_vel_control_9','SimulationMode','normal');

%% OUTPUT PLOTS (CLOSED LOOP)
% SPEED plot
figure
plot (aux.t,aux.w_est_rpm,'b')
hold on
plot (aux.t,aux.r_w,'r')
legend ('Real','Reference')
title ('vel loop')
% CURRENT plot
figure
plot (aux.t,aux.i_m,'b')
title ('current loop')

figure
subplot (2,1,1)
plot (aux.t,aux.w_est_rpm,'b')
hold on
plot (aux.t,aux.r_w,'r')
legend ('Real','Reference')
title ('Velocitat [rad/s]')
subplot (2,1,2)
plot (aux.t,aux.i_m)
title ('Corrent [A]')

%% FAULT FREE ESCENARIO
% RESIDUAL 1 (Electrical)
u1=aux.V-Ke*aux.w_est*2*pi;
i_aprox=predict(sys_el_id,[aux.i_m,u1],t_pred);

r1=aux.i_m-i_aprox;
offset_r1=mean(r1);
r1=r1-offset_r1;

figure
plot (aux.t,r1)
hold on
sigma_1d=1.2*max(abs(r1));
plot (aux.t,ones(size(r1))*sigma_1d,'r')
plot (aux.t,-1*ones(size(r1))*sigma_1d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 1 Fault Free')

% RESIDUAL 2 (Mechanical)
u2=-aux.Tload+Kt*aux.i_m;
w_aprox_wd=predict(sys_mec_wd,[aux.w_est*2*pi,u2],t_pred);

r2_wd=aux.w_est*2*pi-w_aprox_wd;
offset_r2=mean(r2_wd);
r2_wd=r2_wd-offset_r2;

figure
plot (aux.t,r2_wd)
hold on
sigma_2d=1.2*max(abs(r2_wd));
plot (aux.t,ones(size(r2_wd))*sigma_2d,'r')
plot (aux.t,-1*ones(size(r2_wd))*sigma_2d,'r')
xlabel ('Time [s]')
ylabel ('rad/s')
title ('Residual 2 Discr Fault Free')

% RESIDUAL 3 (I(s)/V(s))
tf_r3=tf([J b],[J*L J*R+b*L b*R+Ke*Kt]);
discret_r3=c2d(tf_r3,Tsd,'zoh');

% u3=aux 
aprox_r3=predict(discret_r3,[aux.i_m,aux.V],t_pred);
r3=aux.i_m-aprox_r3;
offset_r3=mean(r3);
r3=r3-offset_r3;

figure
plot (aux.t,r3)
hold on
sigma_3d=1.2*max(abs(r3));
plot (aux.t,ones(size(r3))*sigma_3d,'r')
plot (aux.t,-1*ones(size(r3))*sigma_3d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 3 Discret Faul Free')

% RESIDUAL 4 (W(s)/V(s))
tf_r4=tf(Kt,[J*L J*R+b*L b*R+Ke*Kt]);
discret_r4=c2d(tf_r4,Tsd,'zoh');

aprox_r4=predict(discret_r4,[aux.w_est*2*pi,aux.V],t_pred);
r4=aux.w_est*2*pi-aprox_r4;
offset_r4=mean(r4);
r4=r4-offset_r4;

figure
plot (aux.t,r4)
hold on
sigma_4d=1.2*max(abs(r4));
plot (aux.t,ones(size(r4))*sigma_4d,'r')
plot (aux.t,-1*ones(size(r4))*sigma_4d,'r')
xlabel ('Time [s]')
ylabel ('rad/s')
title ('Residual 4 Discret Faul Free')

%% R_fault setting
ramp_f_R=abrup_incip;
R_fault=R;
vel_R=0.01;
tol_R=0.01;
% Disabled other fault parameters disabled 
J_fault=0;
ramp_f_J=0;
vel_J=0.5;
pos_stuck=0;
stuck_duration=1;
f_i=0;
vel_i=0;
ramp_i=0;
ramp_f_L=0;
L_fault=0;
vel_L=0.1;
ramp_f_B=0;
B_fault=0;
vel_B=0.001
ramp_f_load=0;
vel_Load=0.1;
Load_fault=0;
delta_load=0.01;
ramp_f_Vdc=0;
Vdc_fault=0;
vel_Vdc=0.01;
% Run updated system 
open_system('Simple_model_close_loop_vel_control_9');
aux=sim('Simple_model_close_loop_vel_control_9','SimulationMode','normal');

%% Residual evaluation with R_fault
figure
subplot (4,1,1)
plot (aux.t,aux.V)
title ('Voltage [V]')
subplot (4,1,2)
plot (aux.t,aux.i_m)
title ('Current [A]')
subplot (4,1,3)
plot (aux.t,aux.theta)
title ('Position [rad]')
subplot (4,1,4)
plot (aux.t,aux.w_est*2*pi)
title ('Velocity [rad/s] estimated with position sensor')
%Residual 1
u1=aux.V-Ke*aux.w_est*2*pi;
i_aprox=predict(sys_el_id,[aux.i_m,u1],t_pred);
r1=aux.i_m-i_aprox;
%r1=r1-offset_r1;

figure
plot (aux.t,r1)
hold on 
plot (aux.t,ones(size(r1))*sigma_1d,'r')
plot (aux.t,-1*ones(size(r1))*sigma_1d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 1 R Fault')

%Residual 2
u2=-aux.Tload+Kt*aux.i_m;
w_aprox_wd=predict(sys_mec_wd,[aux.w_est*2*pi,u2],t_pred);

r2=aux.w_est*2*pi-w_aprox_wd;
r2=r2-offset_r2;

figure
plot (aux.t,r2)
hold on 
plot (aux.t,ones(size(r2))*sigma_2d,'r')
plot (aux.t,-1*ones(size(r2))*sigma_2d,'r')
xlabel ('Time [s]')
ylabel ('rad/s')
title ('Residual 2 R Fault')

%Residual 3
aprox_r3=predict(discret_r3,[aux.i_m,aux.V],t_pred);
r3=aux.i_m-aprox_r3;
r3=r3-offset_r3;

figure
plot (aux.t,r3)
hold on
plot (aux.t,ones(size(r3))*sigma_3d,'r')
plot (aux.t,-1*ones(size(r3))*sigma_3d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 3 R Fault')

%Residual 4
aprox_r4=predict(discret_r4,[aux.w_est*2*pi,aux.V],t_pred);
r4=aux.w_est*2*pi-aprox_r4;
r4=r4-offset_r4;

figure
plot (aux.t,r4)
hold on
plot (aux.t,ones(size(r4))*sigma_4d,'r')
plot (aux.t,-1*ones(size(r4))*sigma_4d,'r')
xlabel ('Time [s]')
ylabel ('rad/s')
title ('Residual 4 R Fault')

% R_fault evolution
figure 
plot (aux.t,aux.V_R_fault)
title (' R fault f_R')
xlabel ('Time [s]')
ylabel ('Ohms \Omega')

% Save R_fault data
V_R_fault=aux.V_R_fault;
i_m = aux.i_m;
w_m = aux.w_est*2*pi;
if abrup_incip == 0
    save dades_R_Fault_abrup r1 r2 r3 r4 V_R_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
else
    save dades_R_Fault_incip r1 r2 r3 r4 V_R_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
end

%% J_fault setting
J_fault=100*J;
ramp_f_J=abrup_incip;
vel_J=5;
% Disabled other fault parameters disabled 
R_fault=0;
ramp_f_R=0;
vel_R=0.001;
pos_stuck=0;
stuck_duration=1;
f_i=0;
vel_i=0;
ramp_i=0;
ramp_f_L=0;
L_fault=0;
vel_L=0.1;
ramp_f_B=0;
B_fault=0;
vel_B=0.001;
ramp_f_Vdc=0;
Vdc_fault=0;
vel_Vdc=0.01;
ramp_f_load=0;
vel_Load=0.1;
Load_fault=0;
delta_load=0.01;
% Run updated system 
open_system('Simple_model_close_loop_vel_control_9');
aux=sim('Simple_model_close_loop_vel_control_9','SimulationMode','normal');

%% Residual evaluation with J_fault
figure
subplot (4,1,1)
plot (aux.t,aux.V)
title ('Voltage [V]')
subplot (4,1,2)
plot (aux.t,aux.i_m)
title ('Current [A]')
subplot (4,1,3)
plot (aux.t,aux.theta)
title ('Position [rad]')
subplot (4,1,4)
plot (aux.t,aux.w_est*2*pi)
title ('Velocity [rad/s] estimated with position sensor')
%Residual 1
u1=aux.V-Ke*aux.w_est*2*pi;
i_aprox=predict(sys_el_id,[aux.i_m,u1],t_pred);
r1=aux.i_m-i_aprox;

figure
plot (aux.t,r1)
hold on 
plot (aux.t,ones(size(r1))*sigma_1d,'r')
plot (aux.t,-1*ones(size(r1))*sigma_1d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 1 J Fault')

%Residual 2
u2=-aux.Tload+Kt*aux.i_m;
w_aprox_wd=predict(sys_mec_wd,[aux.w_est*2*pi,u2],t_pred);

r2=aux.w_est*2*pi-w_aprox_wd;

figure
plot (aux.t,r2)
hold on 
plot (aux.t,ones(size(r2))*sigma_2d,'r')
plot (aux.t,-1*ones(size(r2))*sigma_2d,'r')
xlabel ('Time [s]')
ylabel ('rad/S')
title ('Residual 2 J Fault')

%Residual 3
aprox_r3=predict(discret_r3,[aux.i_m,aux.V],t_pred);
r3=aux.i_m-aprox_r3;

figure
plot (aux.t,r3)
hold on
plot (aux.t,ones(size(r3))*sigma_3d,'r')
plot (aux.t,-1*ones(size(r3))*sigma_3d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 3 J Fault')

%Residual 4
aprox_r4=predict(discret_r4,[aux.w_est*2*pi,aux.V],t_pred);
r4=aux.w_est*2*pi-aprox_r4;

figure
plot (aux.t,r4)
hold on
plot (aux.t,ones(size(r4))*sigma_4d,'r')
plot (aux.t,-1*ones(size(r4))*sigma_4d,'r')
xlabel ('Time [s]')
ylabel ('rad/s')
title ('Residual 4 J Fault')

% J_fault evolution
figure 
plot (aux.t,aux.V_J_fault)
title (' J fault f_J')
xlabel ('Time [s]')
ylabel (' Kg·m^2 ')

% Save J_fault data
V_J_fault=aux.V_J_fault;
i_m = aux.i_m;
w_m = aux.w_est*2*pi;
if abrup_incip == 0
    save dades_J_Fault_abrup r1 r2 r3 r4 V_J_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
else
    save dades_J_Fault_incip r1 r2 r3 r4 V_J_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
end

%% stuck pos sensor
pos_stuck=1;
stuck_duration=1;
% Disabled other fault parameters disabled 
R_fault=0;
ramp_f_R=0;
vel_R=0.001;
J_fault=0;
ramp_f_J=0;
vel_J=0.5;
f_i=0;
vel_i=0;
ramp_i=0;
ramp_f_L=0;
L_fault=0;
vel_L=0.1;
ramp_f_B=0;
B_fault=0;
vel_B=0.001;
ramp_f_Vdc=0;
Vdc_fault=0;
vel_Vdc=0.01;
ramp_f_load=0;
vel_Load=0.1;
Load_fault=0;
delta_load=0.01;
% Run updated system 
open_system('Simple_model_close_loop_vel_control_9');
aux=sim('Simple_model_close_loop_vel_control_9','SimulationMode','normal');

%% Residual evaluation with stuck_pos_sensor fault
figure
subplot (4,1,1)
plot (aux.t,aux.V)
title ('Voltage [V]')
subplot (4,1,2)
plot (aux.t,aux.i_m)
title ('Current [A]')
subplot (4,1,3)
plot (aux.t,aux.theta)
title ('Position [rad]')
subplot (4,1,4)
plot (aux.t,aux.w_est*2*pi)
title ('Velocity [rad/s] estimated with position sensor')
%Residual 1
u1=aux.V-Ke*aux.w_est*2*pi;
i_aprox=predict(sys_el_id,[aux.i_m,u1],t_pred);
r1=aux.i_m-i_aprox;

figure
plot (aux.t,r1)
hold on 
plot (aux.t,ones(size(r1))*sigma_1d,'r')
plot (aux.t,-1*ones(size(r1))*sigma_1d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 1 Stuck pos sensor Fault')
%Residual 2
u2=-aux.Tload+Kt*aux.i_m;
w_aprox_wd=predict(sys_mec_wd,[aux.w_est*2*pi,u2],t_pred);

r2=aux.w_est*2*pi-w_aprox_wd;

figure
plot (aux.t,r2)
hold on 
plot (aux.t,ones(size(r2))*sigma_2d,'r')
plot (aux.t,-1*ones(size(r2))*sigma_2d,'r')
xlabel ('Time [s]')
ylabel ('rad/S')
title ('Residual 2 Stuck pos sensor Fault')

%Residual 3
aprox_r3=predict(discret_r3,[aux.i_m,aux.V],t_pred);
r3=aux.i_m-aprox_r3;

figure
plot (aux.t,r3)
hold on
plot (aux.t,ones(size(r3))*sigma_3d,'r')
plot (aux.t,-1*ones(size(r3))*sigma_3d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 3 Stuck pos sensor Fault')

%Residual 4
aprox_r4=predict(discret_r4,[aux.w_est*2*pi,aux.V],t_pred);
r4=aux.w_est*2*pi-aprox_r4;

figure
plot (aux.t,r4)
hold on
plot (aux.t,ones(size(r4))*sigma_4d,'r')
plot (aux.t,-1*ones(size(r4))*sigma_4d,'r')
xlabel ('Time [s]')
ylabel ('rad/s')
title ('Residual 4 Stuck pos sensor Fault')

%Pos_stuck fault evolution
figure 
subplot (2,1,1)
plot (aux.t,aux.theta_real_p,'b',aux.t,aux.theta_est_p,'r')
legend ('Real','Estimated')
title (' Stuck pos sensor Fault')
xlabel ('Time [s]')
ylabel ('Counts')
subplot (2,1,2)
plot (aux.t,aux.theta_real_p-aux.theta_est_p,'r')
xlabel ('Time [s]')
ylabel ('Counts')
title ('Count error')

%Save Stuckpos_data
V_stuck_fault=aux.theta_real_p-aux.theta_est_p;
i_m = aux.i_m;
w_m = aux.w_est*2*pi;
if abrup_incip == 0
    save dades_stuckpos_Fault_abrup r1 r2 r3 r4 V_stuck_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
else
    save dades_stuckpos_Fault_incip r1 r2 r3 r4 V_stuck_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
end


%% Current sensor fault
f_i=1;
vel_i=0.01;
ramp_i=abrup_incip;
% Disabled other fault parameters disabled 
R_fault=0;
ramp_f_R=0;
vel_R=0.001;
J_fault=0;
ramp_f_J=0;
vel_J=0.5;
pos_stuck=0;
stuck_duration=1;
ramp_f_L=0;
L_fault=0;
vel_L=0.1;
ramp_f_B=0;
B_fault=0;
vel_B=0.001;
ramp_f_Vdc=0;
Vdc_fault=0;
vel_Vdc=0.01;
ramp_f_load=0;
vel_Load=0.1;
Load_fault=0;
delta_load=0.01;
% Run updated system 
open_system('Simple_model_close_loop_vel_control_9');
aux=sim('Simple_model_close_loop_vel_control_9','SimulationMode','normal');

%% Residual evaluation with current sensor fault
figure
subplot (4,1,1)
plot (aux.t,aux.V)
title ('Voltage [V]')
subplot (4,1,2)
plot (aux.t,aux.i_m)
title ('Current [A]')
subplot (4,1,3)
plot (aux.t,aux.theta)
title ('Position [rad]')
subplot (4,1,4)
plot (aux.t,aux.w_est*2*pi)
title ('Velocity [rad/s] estimated with position sensor')
%Residual 1
u1=aux.V-Ke*aux.w_est*2*pi;
i_aprox=predict(sys_el_id,[aux.i_m,u1],t_pred);
r1=aux.i_m-i_aprox;

figure
plot (aux.t,r1)
hold on 
plot (aux.t,ones(size(r1))*sigma_1d,'r')
plot (aux.t,-1*ones(size(r1))*sigma_1d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 1 Current sensor Fault')

%Residual 2
u2=-aux.Tload+Kt*aux.i_m;
w_aprox_wd=predict(sys_mec_wd,[aux.w_est*2*pi,u2],t_pred);

r2=aux.w_est*2*pi-w_aprox_wd;

figure
plot (aux.t,r2)
hold on 
plot (aux.t,ones(size(r2))*sigma_2d,'r')
plot (aux.t,-1*ones(size(r2))*sigma_2d,'r')
xlabel ('Time [s]')
ylabel ('rad/S')
title ('Residual 2  Current sensor Fault')

%Residual 3
aprox_r3=predict(discret_r3,[aux.i_m,aux.V],t_pred);
r3=aux.i_m-aprox_r3;

figure
plot (aux.t,r3)
hold on
plot (aux.t,ones(size(r3))*sigma_3d,'r')
plot (aux.t,-1*ones(size(r3))*sigma_3d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 3 Current sensor Fault')

%Residual 4
aprox_r4=predict(discret_r4,[aux.w_est*2*pi,aux.V],t_pred);
r4=aux.w_est*2*pi-aprox_r4;

figure
plot (aux.t,r4)
hold on
plot (aux.t,ones(size(r4))*sigma_4d,'r')
plot (aux.t,-1*ones(size(r4))*sigma_4d,'r')
xlabel ('Time [s]')
ylabel ('rad/s')
title ('Residual 4 Current sensor Fault')

% isensor_fault evolution
figure 
plot (aux.t,aux.V_isensor_fault)
axis([0 100 0 1.1])
title (' Current sensor fault f_i')
xlabel ('Time [s]')
ylabel ('Amper A')

% Save isensor_fault data
V_isensor_fault=aux.V_isensor_fault;
i_m = aux.i_m;
w_m = aux.w_est*2*pi;
if abrup_incip == 0
    save dades_isensor_Fault_abrup r1 r2 r3 r4 V_isensor_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
else
    save dades_isensor_Fault_incip r1 r2 r3 r4 V_isensor_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
end

%% L_fault setting
ramp_f_L=abrup_incip;
L_fault=L*10;
vel_L=0.1;
% Disabled other fault parameters disabled 
R_fault=0;
ramp_f_R=0;
vel_R=0.001;
J_fault=0;
ramp_f_J=0;
vel_J=0.5;
pos_stuck=0;
stuck_duration=1;
f_i=0;
vel_i=0;
ramp_i=0;
ramp_f_B=0;
B_fault=0;
vel_B=0.001;
ramp_f_Vdc=0;
Vdc_fault=0;
vel_Vdc=0.01;
ramp_f_load=0;
vel_Load=0.1;
Load_fault=0;
delta_load=0.01;
% Run updated system 
open_system('Simple_model_close_loop_vel_control_9');
aux=sim('Simple_model_close_loop_vel_control_9','SimulationMode','normal');

%% Residual evaluation with L_fault
figure
subplot (4,1,1)
plot (aux.t,aux.V)
title ('Voltage [V]')
subplot (4,1,2)
plot (aux.t,aux.i_m)
title ('Current [A]')
subplot (4,1,3)
plot (aux.t,aux.theta)
title ('Position [rad]')
subplot (4,1,4)
plot (aux.t,aux.w_est*2*pi)
title ('Velocity [rad/s] estimated with position sensor')
%Residual 1
u1=aux.V-Ke*aux.w_est*2*pi;
i_aprox=predict(sys_el_id,[aux.i_m,u1],t_pred);
r1=aux.i_m-i_aprox;

figure
plot (aux.t,r1)
hold on 
plot (aux.t,ones(size(r1))*sigma_1d,'r')
plot (aux.t,-1*ones(size(r1))*sigma_1d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 1 L Fault')

%Residual 2
u2=-aux.Tload+Kt*aux.i_m;
w_aprox_wd=predict(sys_mec_wd,[aux.w_est*2*pi,u2],t_pred);

r2=aux.w_est*2*pi-w_aprox_wd;

figure
plot (aux.t,r2)
hold on 
plot (aux.t,ones(size(r2))*sigma_2d,'r')
plot (aux.t,-1*ones(size(r2))*sigma_2d,'r')
xlabel ('Time [s]')
ylabel ('rad/S')
title ('Residual 2 L Fault')

%Residual 3
aprox_r3=predict(discret_r3,[aux.i_m,aux.V],t_pred);
r3=aux.i_m-aprox_r3;

figure
plot (aux.t,r3)
hold on
plot (aux.t,ones(size(r3))*sigma_3d,'r')
plot (aux.t,-1*ones(size(r3))*sigma_3d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 3 L Fault')

%Residual 4
aprox_r4=predict(discret_r4,[aux.w_est*2*pi,aux.V],t_pred);
r4=aux.w_est*2*pi-aprox_r4;

figure
plot (aux.t,r4)
hold on
plot (aux.t,ones(size(r4))*sigma_4d,'r')
plot (aux.t,-1*ones(size(r4))*sigma_4d,'r')
xlabel ('Time [s]')
ylabel ('rad/s')
title ('Residual 4 L Fault')

% L_fault evolution
figure 
plot (aux.t,aux.V_L_fault)
title (' L fault f_L')
xlabel ('Time [s]')
ylabel ('Henry H')

% Save L_fault data
V_L_fault=aux.V_L_fault;
i_m = aux.i_m;
w_m = aux.w_est*2*pi;
if abrup_incip == 0
    save dades_L_Fault_abrup r1 r2 r3 r4 V_L_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
else
    save dades_L_Fault_incip r1 r2 r3 r4 V_L_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
end

%% B_fault setting
ramp_f_B=abrup_incip;
B_fault=2*b;
vel_B=0.1;
% Disabled other fault parameters disabled 
R_fault=0;
ramp_f_R=0;
vel_R=0.001;
J_fault=0;
ramp_f_J=0;
vel_J=0.5;
pos_stuck=0;
stuck_duration=1;
f_i=0;
vel_i=0;
ramp_i=0;
ramp_f_L=0;
L_fault=0;
vel_L=0.1;
ramp_f_Vdc=0;
Vdc_fault=0;
vel_Vdc=0.01;
ramp_f_load=0;
vel_Load=0.1;
Load_fault=0;
delta_load=0.01;
% Run updated system 
open_system('Simple_model_close_loop_vel_control_9');
aux=sim('Simple_model_close_loop_vel_control_9','SimulationMode','normal');

%% Residual evaluation with B_fault
figure
subplot (4,1,1)
plot (aux.t,aux.V)
title ('Voltage [V]')
subplot (4,1,2)
plot (aux.t,aux.i_m)
title ('Current [A]')
subplot (4,1,3)
plot (aux.t,aux.theta)
title ('Position [rad]')
subplot (4,1,4)
plot (aux.t,aux.w_est*2*pi)
title ('Velocity [rad/s] estimated with position sensor')
%Residual 1
u1=aux.V-Ke*aux.w_est*2*pi;
i_aprox=predict(sys_el_id,[aux.i_m,u1],t_pred);
r1=aux.i_m-i_aprox;

figure
plot (aux.t,r1)
hold on 
plot (aux.t,ones(size(r1))*sigma_1d,'r')
plot (aux.t,-1*ones(size(r1))*sigma_1d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 1 B Fault')

%Residual 2
u2=-aux.Tload+Kt*aux.i_m;
w_aprox_wd=predict(sys_mec_wd,[aux.w_est*2*pi,u2],t_pred);

r2=aux.w_est*2*pi-w_aprox_wd;

figure
plot (aux.t,r2)
hold on 
plot (aux.t,ones(size(r2))*sigma_2d,'r')
plot (aux.t,-1*ones(size(r2))*sigma_2d,'r')
xlabel ('Time [s]')
ylabel ('rad/S')
title ('Residual 2 B Fault')

%Residual 3
aprox_r3=predict(discret_r3,[aux.i_m,aux.V],t_pred);
r3=aux.i_m-aprox_r3;

figure
plot (aux.t,r3)
hold on
plot (aux.t,ones(size(r3))*sigma_3d,'r')
plot (aux.t,-1*ones(size(r3))*sigma_3d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 3 B Fault')

%Residual 4
aprox_r4=predict(discret_r4,[aux.w_est*2*pi,aux.V],t_pred);
r4=aux.w_est*2*pi-aprox_r4;

figure
plot (aux.t,r4)
hold on
plot (aux.t,ones(size(r4))*sigma_4d,'r')
plot (aux.t,-1*ones(size(r4))*sigma_4d,'r')
xlabel ('Time [s]')
ylabel ('rad/s')
title ('Residual 4 B Fault')

% B_fault evolution
figure 
plot (aux.t,aux.V_B_fault)
title (' B fault f_B ')
xlabel (' Time [s] ')
ylabel ('(N·m·s)/rad ')

% Save B_fault data
V_B_fault=aux.V_B_fault;
i_m = aux.i_m;
w_m = aux.w_est*2*pi;
if abrup_incip == 0
    save dades_B_Fault_abrup r1 r2 r3 r4 V_B_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
else
    save dades_B_Fault_incip r1 r2 r3 r4 V_B_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
end

%% Vdc_fault setting
ramp_f_Vdc=abrup_incip;
Vdc_fault=7;
vel_Vdc=0.1;
% Disabled other fault parameters disabled 
R_fault=0;
ramp_f_R=0;
vel_R=0.001;
J_fault=0;
ramp_f_J=0;
vel_J=0.5;
pos_stuck=0;
stuck_duration=1;
f_i=0;
vel_i=0;
ramp_i=0;
ramp_f_L=0;
L_fault=0;
vel_L=0.1;
ramp_f_B=0;
B_fault=0;
vel_B=0.001;
ramp_f_load=0;
vel_Load=0.1;
Load_fault=0;
delta_load=0.01;
% Run updated system 
open_system('Simple_model_close_loop_vel_control_9');
aux=sim('Simple_model_close_loop_vel_control_9','SimulationMode','normal');

%% Residual evaluation with Vdc_fault
figure
subplot (4,1,1)
plot (aux.t,aux.V)
title ('Voltage [V]')
subplot (4,1,2)
plot (aux.t,aux.i_m)
title ('Current [A]')
subplot (4,1,3)
plot (aux.t,aux.theta)
title ('Position [rad]')
subplot (4,1,4)
plot (aux.t,aux.w_est*2*pi)
title ('Velocity [rad/s] estimated with position sensor')
%Residual 1
u1=aux.V-Ke*aux.w_est*2*pi;
i_aprox=predict(sys_el_id,[aux.i_m,u1],t_pred);
r1=aux.i_m-i_aprox;

figure
plot (aux.t,r1)
hold on 
plot (aux.t,ones(size(r1))*sigma_1d,'r')
plot (aux.t,-1*ones(size(r1))*sigma_1d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 1 Vdc Fault')

%Residual 2
u2=-aux.Tload+Kt*aux.i_m;
w_aprox_wd=predict(sys_mec_wd,[aux.w_est*2*pi,u2],t_pred);

r2=aux.w_est*2*pi-w_aprox_wd;

figure
plot (aux.t,r2)
hold on 
plot (aux.t,ones(size(r2))*sigma_2d,'r')
plot (aux.t,-1*ones(size(r2))*sigma_2d,'r')
xlabel ('Time [s]')
ylabel ('rad/S')
title ('Residual 2 Vdc Fault')

%Residual 3
aprox_r3=predict(discret_r3,[aux.i_m,aux.V],t_pred);
r3=aux.i_m-aprox_r3;

figure
plot (aux.t,r3)
hold on
plot (aux.t,ones(size(r3))*sigma_3d,'r')
plot (aux.t,-1*ones(size(r3))*sigma_3d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 3 Vdc Fault')

%Residual 4
aprox_r4=predict(discret_r4,[aux.w_est*2*pi,aux.V],t_pred);
r4=aux.w_est*2*pi-aprox_r4;

figure
plot (aux.t,r4)
hold on
plot (aux.t,ones(size(r4))*sigma_4d,'r')
plot (aux.t,-1*ones(size(r4))*sigma_4d,'r')
xlabel ('Time [s]')
ylabel ('rad/s')
title ('Residual 4 Vdc Fault')

% Vdc_fault evolution
figure 
plot (aux.t,aux.V_Vdc_fault)
axis([0 100 0 7.5])
title (' Vdc fault f_Vdc ')
xlabel (' Time [s] ')
ylabel (' Volts ')

% Save Vdc_fault data
V_Vdc_fault=aux.V_Vdc_fault;
i_m = aux.i_m;
w_m = aux.w_est*2*pi;
if abrup_incip == 0
    save dades_Vdc_Fault_abrup r1 r2 r3 r4 V_Vdc_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
else
    save dades_Vdc_Fault_incip r1 r2 r3 r4 V_Vdc_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
end

%% Load_fault setting
vel_Load=0.1;
delta_load=0.05;
Load_fault=2*delta_load;
ramp_f_Load=abrup_incip;
% Disabled other fault parameters disabled 
R_fault=0;
ramp_f_R=0;
vel_R=0.001;
J_fault=0;
ramp_f_J=0;
vel_J=0.5;
pos_stuck=0;
stuck_duration=1;
f_i=0;
vel_i=0;
ramp_i=0;
ramp_f_L=0;
L_fault=0;
vel_L=0.1;
ramp_f_B=0;
B_fault=0;
ramp_f_Vdc=0;
Vdc_fault=0;
vel_Vdc=0.001;
% Run updated system 
open_system('Simple_model_close_loop_vel_control_9');
aux=sim('Simple_model_close_loop_vel_control_9','SimulationMode','normal');

%% Residual evaluation with Load_fault
figure
subplot (4,1,1)
plot (aux.t,aux.V)
title ('Voltage [V]')
subplot (4,1,2)
plot (aux.t,aux.i_m)
title ('Current [A]')
subplot (4,1,3)
plot (aux.t,aux.theta)
title ('Position [rad]')
subplot (4,1,4)
plot (aux.t,aux.w_est*2*pi)
title ('Velocity [rad/s] estimated with position sensor')
%Residual 1
u1=aux.V-Ke*aux.w_est*2*pi;
i_aprox=predict(sys_el_id,[aux.i_m,u1],t_pred);
r1=aux.i_m-i_aprox;

figure
plot (aux.t,r1)
hold on 
plot (aux.t,ones(size(r1))*sigma_1d,'r')
plot (aux.t,-1*ones(size(r1))*sigma_1d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 1 Load Fault')

%Residual 2
u2=-aux.Tload+Kt*aux.i_m;
w_aprox_wd=predict(sys_mec_wd,[aux.w_est*2*pi,u2],t_pred);

r2=aux.w_est*2*pi-w_aprox_wd;

figure
plot (aux.t,r2)
hold on 
plot (aux.t,ones(size(r2))*sigma_2d,'r')
plot (aux.t,-1*ones(size(r2))*sigma_2d,'r')
xlabel ('Time [s]')
ylabel ('rad/S')
title ('Residual 2 Load Fault')

%Residual 3
aprox_r3=predict(discret_r3,[aux.i_m,aux.V],t_pred);
r3=aux.i_m-aprox_r3;

figure
plot (aux.t,r3)
hold on
plot (aux.t,ones(size(r3))*sigma_3d,'r')
plot (aux.t,-1*ones(size(r3))*sigma_3d,'r')
xlabel ('Time [s]')
ylabel ('Ampers')
title ('Residual 3 Load Fault')

%Residual 4
aprox_r4=predict(discret_r4,[aux.w_est*2*pi,aux.V],t_pred);
r4=aux.w_est*2*pi-aprox_r4;

figure
plot (aux.t,r4)
hold on
plot (aux.t,ones(size(r4))*sigma_4d,'r')
plot (aux.t,-1*ones(size(r4))*sigma_4d,'r')
xlabel ('Time [s]')
ylabel ('rad/s')
title ('Residual 4 Load Fault')

% Load_fault evolution
figure 
plot (aux.t,aux.V_Load_fault)
title (' Load fault f_Load')
xlabel ('Time [s]')
ylabel ('Newton meter [N·m]')

% Save Load_fault data
V_Load_fault=aux.V_Load_fault;
i_m = aux.i_m;
w_m = aux.w_est*2*pi;
if abrup_incip == 0
    save dades_Load_Fault_abrup r1 r2 r3 r4 V_Load_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
else
    save dades_Load_Fault_incip r1 r2 r3 r4 V_Load_fault sigma_1d sigma_2d sigma_3d sigma_4d i_m w_m
end

%% Save figures
if abrup_incip==0
    mkdir fig_abrup
    for i=1:54
        nombre=sprintf('Imagen_abrupta_%d.png',i);
        nombre=['fig_abrup','\',nombre];
        figure(i)
        hgsave(nombre);
        saveas(gcf,nombre,'png');  
    end
else
    mkdir fig_incip
    for i=1:54
        nombre=sprintf('Imagen_incipient_%d.png',i);
        nombre=['fig_incip','\',nombre];
        figure(i)
        hgsave(nombre);
        saveas(gcf,nombre,'png');
    end
end
