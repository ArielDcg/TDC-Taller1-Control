%% ============================================================
%       CONTROL COMPLETO: ROLL + DIRECCIONAL + LONGITUDINAL
% =============================================================
close all
clearvars -except latmod longmod linmodel
clc

s = tf('s');

%% ============================================================
%% 1. CONTROL LATERAL / ROLL
% =============================================================

fprintf('\n============================================\n');
fprintf('           CONTROL LATERAL / ROLL\n');
fprintf('============================================\n');

% -------------------------------------------------------------
% Modelo lateral
% -------------------------------------------------------------

A_ROLL = latmod.A;
B_ROLL = latmod.B(:,1);

% Salidas: phi y p
C_ROLL = latmod.C([4,2],:);
D_ROLL = latmod.D([4,2],1);

SysLat_ROLL = ss(A_ROLL,B_ROLL,C_ROLL,D_ROLL);

tfSysLat_ROLL = tf(SysLat_ROLL);

% La segunda TF corresponde a p
G_ROLL = tfSysLat_ROLL(1);


% -------------------------------------------------------------
% SAS ROLL
% -------------------------------------------------------------

% Ganancia encontrada mediante lugar de las raíces
K_roll = -0.106;

% SAS
SAS_TF_ROLL = (1/s) * feedback(G_ROLL,-K_roll);

fprintf('\nK_roll = %.4f\n',K_roll);

fprintf('Polos SAS ROLL:\n');
disp(pole(feedback(G_ROLL,-K_roll)));

fprintf('Damping SAS ROLL:\n');
damp(feedback(G_ROLL,-K_roll));


% -------------------------------------------------------------
% PI ROLL
% -------------------------------------------------------------

zero_ROLL = 0.0195;

PI_ROLL = (s + zero_ROLL)/s;

% Lugar de raíces para encontrar Kp
[r,k] = rlocus(SAS_TF_ROLL*PI_ROLL);

Max_K_ROLL = 0;

for i = 1:length(k)

    if all(real(r(:,i)) <= 0)
        Max_K_ROLL = k(i);

    else
        break
    end

end

Kp_ROLL = 0.00284;
Ki_ROLL = zero_ROLL*Kp_ROLL;

fprintf('\nPI ROLL:\n');
fprintf('Kp_ROLL = %.6f\n',Kp_ROLL);
fprintf('Ki_ROLL = %.6f\n',Ki_ROLL);


% PI real
PI_ROLL_REAL = Kp_ROLL;

% Lazo abierto antes del Lead
LAT_ROLL_OL_PI = PI_ROLL_REAL*SAS_TF_ROLL;


% -------------------------------------------------------------
% RED DE ADELANTO ROLL
% -------------------------------------------------------------

wc_req_ROLL = 5;

[mag_ROLL] = bode(LAT_ROLL_OL_PI,wc_req_ROLL);

dec_ROLL = -20*log10(mag_ROLL) - 9;

b_ROLL = 10^(dec_ROLL/10);

T2_ROLL = 1/(sqrt(b_ROLL)*wc_req_ROLL);

Lead_ROLL = ...
    (1 + b_ROLL*T2_ROLL*s) / ...
    (1 + T2_ROLL*s);


% Lazo abierto FINAL ROLL
L_ROLL = LAT_ROLL_OL_PI;

% Lazo cerrado FINAL
T_ROLL = feedback(L_ROLL,1);


%% ============================================================
%% 2. CONTROL DIRECCIONAL / YAW
% =============================================================

fprintf('\n============================================\n');
fprintf('           CONTROL DIRECCIONAL / YAW\n');
fprintf('============================================\n');

% -------------------------------------------------------------
% Modelo direccional
% -------------------------------------------------------------

A_rud = latmod.A;
B_rud = latmod.B(:,2);

% Salida p para SAS
C_r = [0,0,1,0,0];

sys_sas = ss(A_rud,B_rud,C_r,0);


% -------------------------------------------------------------
% SAS
% -------------------------------------------------------------

k_r = -0.174;

K_sas = [0,0,k_r,0,0];

A_sas = A_rud - B_rud*K_sas;

fprintf('\nk_r = %.4f\n',k_r);

fprintf('Polos SAS DIRECCIONAL:\n');
disp(eig(A_sas));

fprintf('Damping SAS DIRECCIONAL:\n');
damp(A_sas);


% -------------------------------------------------------------
% CAS
% -------------------------------------------------------------

C_psi = [0,0,0,0,1];

sys_cas = ss(A_sas,B_rud,C_psi,0);

% PI
a_YAW = 2;

PI_YAW = (s+a_YAW)/s;


% -------------------------------------------------------------
% Ganancias PI
% -------------------------------------------------------------

Kp_YAW = -1.62;
Ki_YAW = Kp_YAW*a_YAW;

fprintf('\nPI DIRECCIONAL:\n');
fprintf('Kp_YAW = %.6f\n',Kp_YAW);
fprintf('Ki_YAW = %.6f\n',Ki_YAW);


% Planta
P_YAW = ss(A_sas,B_rud,C_psi,0);

% PI real
PI_YAW_REAL = Kp_YAW + Ki_YAW/s;

% Lazo abierto antes del Lead
L_YAW_PI = P_YAW*PI_YAW_REAL;


% -------------------------------------------------------------
% RED DE ADELANTO YAW
% -------------------------------------------------------------

wc_req_YAW = 30;

[mag_YAW] = bode(L_YAW_PI,wc_req_YAW);

dec_YAW = -20*log10(mag_YAW) + 5.1;

b_YAW = 10^(dec_YAW/10);

T2_YAW = 1/(sqrt(b_YAW)*wc_req_YAW);

Lead_YAW = ...
    (1+b_YAW*T2_YAW*s) / ...
    (1+T2_YAW*s);


% Lazo abierto FINAL
L_YAW = Lead_YAW*L_YAW_PI;

% Lazo cerrado FINAL
T_YAW = feedback(L_YAW,1);


%% ============================================================
%% 3. CONTROL LONGITUDINAL
% =============================================================

fprintf('\n============================================\n');
fprintf('             CONTROL LONGITUDINAL\n');
fprintf('============================================\n');

% -------------------------------------------------------------
% Modelo longitudinal reducido
% -------------------------------------------------------------

Along = [longmod.A(4,4), longmod.A(4,3);
         longmod.A(3,4), longmod.A(3,3)];

Blong = [longmod.B(4,1);
         longmod.B(3,1)];

Clong = [longmod.C(4,4), longmod.C(4,3);
         longmod.C(3,4), longmod.C(3,3)];

Dlong = [longmod.D(4,1);
         longmod.D(3,1)];

SysLong = ss(Along,Blong,Clong,Dlong);

tfSysLong = tf(SysLong);

% -------------------------------------------------------------
% Plantas utilizadas
% -------------------------------------------------------------

G2 = tfSysLong(2);
G6 = tf(longmod);

G6 = G6(3,1);


% -------------------------------------------------------------
% SAS
% -------------------------------------------------------------

D_LONG = -0.469;

SASG2 = (1/s)*feedback(G2,D_LONG);
SASG6 = (1/s)*feedback(G6,D_LONG);

fprintf('\nD_LONG = %.4f\n',D_LONG);

fprintf('\nPolos SAS LONGITUDINAL G2:\n');
disp(pole(SASG2));

fprintf('\nPolos SAS LONGITUDINAL G6:\n');
disp(pole(SASG6));


% -------------------------------------------------------------
% PI
% -------------------------------------------------------------

kp_LONG = -14;
ki_LONG = -68;

PI_LONG = kp_LONG + ki_LONG/s;

fprintf('\nPI LONGITUDINAL:\n');
fprintf('Kp_LONG = %.6f\n',kp_LONG);
fprintf('Ki_LONG = %.6f\n',ki_LONG);


% Lazo abierto antes del Lead
KG2 = PI_LONG*SASG2;
KG6 = PI_LONG*SASG6;


% -------------------------------------------------------------
% RED DE ADELANTO
% -------------------------------------------------------------

wc_req_LONG = 29.5;

[mag_LONG] = bode(KG6,wc_req_LONG);

dec_LONG = -20*log10(mag_LONG) + 1.6;

b_LONG = 10^(dec_LONG/10);

T2_LONG = 1/(sqrt(b_LONG)*wc_req_LONG);

Lead_LONG = ...
    (1+b_LONG*T2_LONG*s) / ...
    (1+T2_LONG*s);


% Lazo abierto FINAL
L_LONG = Lead_LONG*KG6;

% Lazo cerrado FINAL
T_LONG = feedback(L_LONG,1);


%% ============================================================
%% 4. PARAMETROS DE LAS REDES DE ADELANTO
% =============================================================

fprintf('\n\n============================================\n');
fprintf('       PARAMETROS REDES DE ADELANTO\n');
fprintf('============================================\n');


%% ROLL

Kd_ROLL = b_ROLL*T2_ROLL;

fprintf('\n----- ROLL -----\n');
fprintf('b       = %.8f\n',b_ROLL);
fprintf('T2      = %.8f\n',T2_ROLL);
fprintf('Kd_Lead = %.8f\n',Kd_ROLL);

fprintf('\nLead ROLL:\n');
disp(Lead_ROLL);


%% YAW

Kd_YAW = b_YAW*T2_YAW;

fprintf('\n----- YAW -----\n');
fprintf('b       = %.8f\n',b_YAW);
fprintf('T2      = %.8f\n',T2_YAW);
fprintf('Kd_Lead = %.8f\n',Kd_YAW);

fprintf('\nLead YAW:\n');
disp(Lead_YAW);


%% LONGITUDINAL

Kd_LONG = b_LONG*T2_LONG;

fprintf('\n----- LONGITUDINAL -----\n');
fprintf('b       = %.8f\n',b_LONG);
fprintf('T2      = %.8f\n',T2_LONG);
fprintf('Kd_Lead = %.8f\n',Kd_LONG);

fprintf('\nLead LONGITUDINAL:\n');
disp(Lead_LONG);


%% ============================================================
%% 5. NUMERADORES Y DENOMINADORES PARA SIMULINK
% =============================================================

fprintf('\n\n============================================\n');
fprintf('      PARAMETROS PARA SIMULINK\n');
fprintf('============================================\n');


%% -------------------------------------------------------------
% ROLL
% -------------------------------------------------------------

% Controlador completo PI + Lead
CTRL_ROLL = Lead_ROLL*PI_ROLL_REAL;

[Num_ROLL,Den_ROLL] = tfdata(CTRL_ROLL,'v');

fprintf('\n===== ROLL =====\n');

fprintf('Numerador:\n');
disp(Num_ROLL);

fprintf('Denominador:\n');
disp(Den_ROLL);

fprintf('Kp = %.8f\n',Kp_ROLL);
fprintf('Ki = %.8f\n',Ki_ROLL);
fprintf('Kd_Lead = %.8f\n',Kd_ROLL);


%% -------------------------------------------------------------
% YAW
% -------------------------------------------------------------

CTRL_YAW = Lead_YAW*PI_YAW_REAL;

[Num_YAW,Den_YAW] = tfdata(CTRL_YAW,'v');

fprintf('\n===== YAW / DIRECCIONAL =====\n');

fprintf('Numerador:\n');
disp(Num_YAW);

fprintf('Denominador:\n');
disp(Den_YAW);

fprintf('Kp = %.8f\n',Kp_YAW);
fprintf('Ki = %.8f\n',Ki_YAW);
fprintf('Kd_Lead = %.8f\n',Kd_YAW);


%% -------------------------------------------------------------
% LONGITUDINAL
% -------------------------------------------------------------

CTRL_LONG = Lead_LONG*PI_LONG;

[Num_LONG,Den_LONG] = tfdata(CTRL_LONG,'v');

fprintf('\n===== LONGITUDINAL =====\n');

fprintf('Numerador:\n');
disp(Num_LONG);

fprintf('Denominador:\n');
disp(Den_LONG);

fprintf('Kp = %.8f\n',kp_LONG);
fprintf('Ki = %.8f\n',ki_LONG);
fprintf('Kd_Lead = %.8f\n',Kd_LONG);


%% ============================================================
%% 6. BODE DE LOS TRES LAZOS ABIERTOS
% =============================================================

figure('Name','Bode - Lazos abiertos');

subplot(3,1,1)
bode(L_ROLL)
grid on
title('Lazo abierto - ROLL')

subplot(3,1,2)
bode(L_YAW)
grid on
title('Lazo abierto - DIRECCIONAL / YAW')

subplot(3,1,3)
bode(L_LONG)
grid on
title('Lazo abierto - LONGITUDINAL')


%% ============================================================
%% 7. RESPUESTAS AL ESCALON
% =============================================================

figure('Name','Respuesta al escalon');

subplot(3,1,1)
step(T_ROLL)
grid on
title('Respuesta al escalon - ROLL')

subplot(3,1,2)
step(T_YAW)
grid on
title('Respuesta al escalon - DIRECCIONAL / YAW')

subplot(3,1,3)
step(T_LONG)
grid on
title('Respuesta al escalon - LONGITUDINAL')


%% ============================================================
%% 8. RESPUESTAS AL ESCALON SUPERPUESTAS
% =============================================================

figure('Name','Respuestas al escalon - Comparacion');

step(T_ROLL)
hold on
step(T_YAW)
step(T_LONG)

grid on

legend('ROLL','DIRECCIONAL / YAW','LONGITUDINAL')

title('Comparacion de respuestas al escalon')

hold off


%% ============================================================
%% 9. ANCHO DE BANDA
% =============================================================

BW_ROLL = bandwidth(T_ROLL);
BW_YAW = bandwidth(T_YAW);
BW_LONG = bandwidth(T_LONG);

fprintf('\n\n============================================\n');
fprintf('             ANCHO DE BANDA\n');
fprintf('============================================\n');

fprintf('\nROLL:\n');
fprintf('%.4f rad/s\n',BW_ROLL);
fprintf('%.4f Hz\n',BW_ROLL/(2*pi));

fprintf('\nDIRECCIONAL / YAW:\n');
fprintf('%.4f rad/s\n',BW_YAW);
fprintf('%.4f Hz\n',BW_YAW/(2*pi));

fprintf('\nLONGITUDINAL:\n');
fprintf('%.4f rad/s\n',BW_LONG);
fprintf('%.4f Hz\n',BW_LONG/(2*pi));


%% ============================================================
%% 10. RESUMEN FINAL
% =============================================================

fprintf('\n\n============================================\n');
fprintf('                 RESUMEN\n');
fprintf('============================================\n');

fprintf('\n                 Kp          Ki          Kd_Lead\n');
fprintf('ROLL       %10.4f  %10.4f  %10.4f\n', ...
    Kp_ROLL,Ki_ROLL,Kd_ROLL);

fprintf('YAW        %10.4f  %10.4f  %10.4f\n', ...
    Kp_YAW,Ki_YAW,Kd_YAW);

fprintf('LONG       %10.4f  %10.4f  %10.4f\n', ...
    kp_LONG,ki_LONG,Kd_LONG);

fprintf('\nAnchos de banda:\n');

fprintf('ROLL       %10.4f rad/s   %10.4f Hz\n', ...
    BW_ROLL,BW_ROLL/(2*pi));

fprintf('YAW        %10.4f rad/s   %10.4f Hz\n', ...
    BW_YAW,BW_YAW/(2*pi));

fprintf('LONG       %10.4f rad/s   %10.4f Hz\n', ...
    BW_LONG,BW_LONG/(2*pi));