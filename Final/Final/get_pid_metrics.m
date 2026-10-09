load('modelo_lin.mat');
s = tf('s');

fprintf('===================================================\n');
fprintf('           PID ROLL / LATERAL\n');
fprintf('===================================================\n');

A_ROLL = latmod.A;
B_ROLL = latmod.B(:,1);
C_ROLL = latmod.C([4,2],:);
D_ROLL = latmod.D([4,2],1);
SysLat_ROLL = ss(A_ROLL,B_ROLL,C_ROLL,D_ROLL);
tfSysLat_ROLL = tf(SysLat_ROLL);
G_ROLL = tfSysLat_ROLL(2);

K_roll = -0.106;
SAS_TF_ROLL = (1/s) * feedback(G_ROLL,-K_roll);
zero_ROLL = 0.0195;
PI_ROLL = (s + zero_ROLL)/s;

Kp_ROLL = 0.00284;
Ki_ROLL = zero_ROLL*Kp_ROLL;
PI_ROLL_REAL = Kp_ROLL*PI_ROLL;
LAT_ROLL_OL_PI = PI_ROLL_REAL*SAS_TF_ROLL;

wc_req_ROLL = 5;
[mag_ROLL] = bode(LAT_ROLL_OL_PI,wc_req_ROLL);
dec_ROLL = -20*log10(mag_ROLL) - 9;
b_ROLL = 10^(dec_ROLL/10);
T2_ROLL = 1/(sqrt(b_ROLL)*wc_req_ROLL);
Lead_ROLL = (1 + b_ROLL*T2_ROLL*s) / (1 + T2_ROLL*s);

L_ROLL = Lead_ROLL*LAT_ROLL_OL_PI;
T_ROLL = feedback(L_ROLL,1);
S_ROLL = feedback(1, L_ROLL);
KS_ROLL = feedback(Lead_ROLL*PI_ROLL_REAL, (1/s)*G_ROLL); % or control effort transfer function

bw_pid_roll = bandwidth(T_ROLL);
sinfo_pid_roll = stepinfo(T_ROLL);
normS_pid_roll = norm(S_ROLL, Inf);
normKS_pid_roll = norm(feedback(Lead_ROLL*PI_ROLL_REAL, (1/s)*feedback(G_ROLL, -K_roll)), Inf);

fprintf('PID ROLL:\n');
fprintf('BW = %.4f rad/s (%.4f Hz)\n', bw_pid_roll, bw_pid_roll/(2*pi));
fprintf('Overshoot = %.2f %%\n', sinfo_pid_roll.Overshoot);
fprintf('SettlingTime = %.4f s\n', sinfo_pid_roll.SettlingTime);
fprintf('||S||_inf = %.4f, ||KS||_inf = %.4f\n', normS_pid_roll, normKS_pid_roll);

fprintf('===================================================\n');
fprintf('           PID YAW / DIRECCIONAL\n');
fprintf('===================================================\n');

A_rud = latmod.A;
B_rud = latmod.B(:,2);
C_r = [0,0,1,0,0];
k_r = -0.174;
K_sas = [0,0,k_r,0,0];
A_sas = A_rud - B_rud*K_sas;
C_psi = [0,0,0,0,1];
P_YAW = ss(A_sas,B_rud,C_psi,0);

a_YAW = 2;
Kp_YAW = -1.62;
Ki_YAW = Kp_YAW*a_YAW;
PI_YAW_REAL = Kp_YAW + Ki_YAW/s;
L_YAW_PI = P_YAW*PI_YAW_REAL;

wc_req_YAW = 30;
[mag_YAW] = bode(L_YAW_PI,wc_req_YAW);
dec_YAW = -20*log10(mag_YAW) + 5.1;
b_YAW = 10^(dec_YAW/10);
T2_YAW = 1/(sqrt(b_YAW)*wc_req_YAW);
Lead_YAW = (1+b_YAW*T2_YAW*s) / (1+T2_YAW*s);

L_YAW = Lead_YAW*L_YAW_PI;
T_YAW = feedback(L_YAW,1);
S_YAW = feedback(1, L_YAW);
KS_YAW = feedback(Lead_YAW*PI_YAW_REAL, P_YAW);

bw_pid_yaw = bandwidth(T_YAW);
sinfo_pid_yaw = stepinfo(T_YAW);
normS_pid_yaw = norm(S_YAW, Inf);
normKS_pid_yaw = norm(KS_YAW, Inf);

fprintf('PID YAW:\n');
fprintf('BW = %.4f rad/s (%.4f Hz)\n', bw_pid_yaw, bw_pid_yaw/(2*pi));
fprintf('Overshoot = %.2f %%\n', sinfo_pid_yaw.Overshoot);
fprintf('SettlingTime = %.4f s\n', sinfo_pid_yaw.SettlingTime);
fprintf('||S||_inf = %.4f, ||KS||_inf = %.4f\n', normS_pid_yaw, normKS_pid_yaw);

fprintf('===================================================\n');
fprintf('           PID LONGITUDINAL\n');
fprintf('===================================================\n');

G6 = tf(longmod);
G6 = G6(3,1);
D_LONG = -0.469;
SASG6 = (1/s)*feedback(G6,D_LONG);
kp_LONG = -14;
ki_LONG = -68;
PI_LONG = kp_LONG + ki_LONG/s;
KG6 = PI_LONG*SASG6;

wc_req_LONG = 29.5;
[mag_LONG] = bode(KG6,wc_req_LONG);
dec_LONG = -20*log10(mag_LONG) + 1.6;
b_LONG = 10^(dec_LONG/10);
T2_LONG = 1/(sqrt(b_LONG)*wc_req_LONG);
Lead_LONG = (1+b_LONG*T2_LONG*s) / (1+T2_LONG*s);

L_LONG = Lead_LONG*KG6;
T_LONG = feedback(L_LONG,1);
S_LONG = feedback(1, L_LONG);
KS_LONG = feedback(Lead_LONG*PI_LONG, SASG6);

bw_pid_long = bandwidth(T_LONG);
sinfo_pid_long = stepinfo(T_LONG);
normS_pid_long = norm(S_LONG, Inf);
normKS_pid_long = norm(KS_LONG, Inf);

fprintf('PID LONGITUDINAL:\n');
fprintf('BW = %.4f rad/s (%.4f Hz)\n', bw_pid_long, bw_pid_long/(2*pi));
fprintf('Overshoot = %.2f %%\n', sinfo_pid_long.Overshoot);
fprintf('SettlingTime = %.4f s\n', sinfo_pid_long.SettlingTime);
fprintf('||S||_inf = %.4f, ||KS||_inf = %.4f\n', normS_pid_long, normKS_pid_long);
