load('modelo_lin.mat');
out_dir = 'c:/Users/caden/OneDrive/Documentos/Ariel/Carrera/VIII Semestre/TDC/Taller1/Documento/TDC_Taller1/';

% Configuración estética de figuras
set(0, 'DefaultAxesFontSize', 10);
set(0, 'DefaultLineLineWidth', 1.5);

%% =========================================================
%% 1. LONGITUDINAL
%% =========================================================
Anew_long = longmod.A;
Bnew_long = longmod.B(:,1);
Cnew_long = longmod.C([3,4],:);
Dnew_long = longmod.D([3,4],1);

SysLong = ss(Anew_long, Bnew_long, Cnew_long, Dnew_long);
tfSysLong = tf(SysLong);

M1T_long = 3; A1T_long = 0.001; Wb1T_long = 1;
W1T_long = tf([1/M1T_long, Wb1T_long], [1, A1T_long*Wb1T_long]);
W1_long = [0, 0; 0, W1T_long];

W2_long = 60 * tf(1,1);

M3T_long = 0.02; A3T_long = 1; Wb3T_long = 2*pi*80;
W3T_long = tf([1/M3T_long, Wb3T_long], [1, A3T_long*Wb3T_long]);
W3_long = [0, 0; 0, W3T_long];

aug_long = augw(tfSysLong, W1_long, W2_long, W3_long);
[K_long, cl_long, gam_long, info_long] = hinfsyn(aug_long, 2, 1);

S_long = minreal(cl_long(2,2)/W1T_long);
KS_long = minreal(cl_long(3,2)/W2_long);
T_long = minreal(cl_long(5,2)/W3T_long);

% Figura Sigma Longitudinal
f1 = figure('Visible', 'off', 'Position', [100, 100, 600, 420]);
w = logspace(-3, 4, 500);
[svS, wS] = sigma(S_long, w);
[svW1, wW1] = sigma(1/W1T_long, w);
[svKS, wKS] = sigma(KS_long, w);
[svW2, wW2] = sigma(1/W2_long, w);
[svT, wT] = sigma(T_long, w);
[svW3, wW3] = sigma(1/W3T_long, w);

semilogx(w, 20*log10(svS(1,:)), 'b', 'LineWidth', 1.8); hold on;
semilogx(w, 20*log10(svW1(1,:)), 'b--', 'LineWidth', 1.5);
semilogx(w, 20*log10(svKS(1,:)), 'r', 'LineWidth', 1.8);
semilogx(w, 20*log10(svW2(1,:)), 'r--', 'LineWidth', 1.5);
semilogx(w, 20*log10(svT(1,:)), 'm', 'LineWidth', 1.8);
semilogx(w, 20*log10(svW3(1,:)), 'm--', 'LineWidth', 1.5);
grid on;
ylim([-80, 50]);
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
title('Valores Singulares y Cotas - Eje Longitudinal');
legend('\sigma(S)', '\gamma / |W_1|', '\sigma(KS)', '\gamma / |W_2|', '\sigma(T)', '\gamma / |W_3|', 'Location', 'SouthWest');
exportgraphics(f1, [out_dir, 'hinf_long_sigma.png'], 'Resolution', 300);
close(f1);

% Figura Paso Longitudinal
f2 = figure('Visible', 'off', 'Position', [100, 100, 600, 420]);
opt = stepDataOptions('StepAmplitude', 1);
[y, t] = step(T_long, 100, opt);
plot(t, y, 'b', 'LineWidth', 1.8); hold on;
yline(1, 'k--');
grid on;
xlabel('Tiempo [s]');
ylabel('Respuesta \theta(t) [rad]');
title('Respuesta al Escalón Unitario - Cabeceo \theta');
exportgraphics(f2, [out_dir, 'hinf_long_step.png'], 'Resolution', 300);
close(f2);


%% =========================================================
%% 2. DIRECCIONAL / YAW
%% =========================================================
Anew_dir = latmod.A;
Bnew_dir = latmod.B(:,1);
Cnew_dir = latmod.C([3,5],:);
Dnew_dir = latmod.D([3,5],1);

SysDir = ss(Anew_dir, Bnew_dir, Cnew_dir, Dnew_dir);
tfSysDir = tf(SysDir);

M1T_dir = 1.7; A1T_dir = 0.01; Wb1T_dir = 0.5;
W1T_dir = tf([1/M1T_dir, Wb1T_dir], [1, A1T_dir*Wb1T_dir]);
W1_dir = [tf(1e-5), 0; 0, W1T_dir];

W2_dir = 2 * tf(1,1);

M3T_dir = 0.2; A3T_dir = 1.4; Wb3T_dir = 2*pi*300;
W3T_dir = tf([1/M3T_dir, Wb3T_dir], [1, A3T_dir*Wb3T_dir]);
W3_dir = [tf(1e-5), 0; 0, W3T_dir];

aug_dir = augw(tfSysDir, W1_dir, W2_dir, W3_dir);
[K_dir, cl_dir, gam_dir, info_dir] = hinfsyn(aug_dir, 2, 1);

S_dir = minreal(cl_dir(2,2)/W1T_dir);
KS_dir = minreal(cl_dir(3,2)/W2_dir);
T_dir = minreal(cl_dir(5,2)/W3T_dir);

% Figura Sigma Direccional
f3 = figure('Visible', 'off', 'Position', [100, 100, 600, 420]);
w = logspace(-3, 4, 500);
[svS, wS] = sigma(S_dir, w);
[svW1, wW1] = sigma(1/W1T_dir, w);
[svKS, wKS] = sigma(KS_dir, w);
[svW2, wW2] = sigma(1/W2_dir, w);
[svT, wT] = sigma(T_dir, w);
[svW3, wW3] = sigma(1/W3T_dir, w);

semilogx(w, 20*log10(svS(1,:)), 'b', 'LineWidth', 1.8); hold on;
semilogx(w, 20*log10(svW1(1,:)), 'b--', 'LineWidth', 1.5);
semilogx(w, 20*log10(svKS(1,:)), 'r', 'LineWidth', 1.8);
semilogx(w, 20*log10(svW2(1,:)), 'r--', 'LineWidth', 1.5);
semilogx(w, 20*log10(svT(1,:)), 'm', 'LineWidth', 1.8);
semilogx(w, 20*log10(svW3(1,:)), 'm--', 'LineWidth', 1.5);
grid on;
ylim([-80, 50]);
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
title('Valores Singulares y Cotas - Eje Direccional (\psi)');
legend('\sigma(S)', '\gamma / |W_1|', '\sigma(KS)', '\gamma / |W_2|', '\sigma(T)', '\gamma / |W_3|', 'Location', 'SouthWest');
exportgraphics(f3, [out_dir, 'hinf_yaw_sigma.png'], 'Resolution', 300);
close(f3);

% Figura Paso Direccional
f4 = figure('Visible', 'off', 'Position', [100, 100, 600, 420]);
[y, t] = step(T_dir, 10);
plot(t, y, 'b', 'LineWidth', 1.8); hold on;
yline(1, 'k--');
grid on;
xlabel('Tiempo [s]');
ylabel('Respuesta \psi(t) [rad]');
title('Respuesta al Escalón Unitario - Guiñada \psi');
exportgraphics(f4, [out_dir, 'hinf_yaw_step.png'], 'Resolution', 300);
close(f4);


%% =========================================================
%% 3. LATERAL / ROLL
%% =========================================================
Anew_lat = latmod.A;
Bnew_lat = latmod.B(:,1);
Cnew_lat = latmod.C([2,4],:);
Dnew_lat = latmod.D([2,4],1);

SysLat = ss(Anew_lat, Bnew_lat, Cnew_lat, Dnew_lat);
tfSysLat = tf(SysLat);

M1T_lat = 1.5; A1T_lat = 0.001; Wb1T_lat = 15;
W1T_lat = tf([1/M1T_lat, Wb1T_lat], [1, A1T_lat*Wb1T_lat]);
W1_lat = [0, 0; 0, W1T_lat];

W2_lat = 5 * tf(1,1);

M3T_lat = 0.02; A3T_lat = 1; Wb3T_lat = 2*pi*100;
W3T_lat = tf([1/M3T_lat, Wb3T_lat], [1, A3T_lat*Wb3T_lat]);
W3_lat = [0, 0; 0, W3T_lat];

aug_lat = augw(tfSysLat, W1_lat, W2_lat, W3_lat);
[K_lat, cl_lat, gam_lat, info_lat] = hinfsyn(aug_lat, 2, 1);

S_lat = minreal(cl_lat(2,2)/W1T_lat);
KS_lat = minreal(cl_lat(3,2)/W2_lat);
T_lat = minreal(cl_lat(5,2)/W3T_lat);

% Figura Sigma Lateral
f5 = figure('Visible', 'off', 'Position', [100, 100, 600, 420]);
w = logspace(-3, 4, 500);
[svS, wS] = sigma(S_lat, w);
[svW1, wW1] = sigma(1/W1T_lat, w);
[svKS, wKS] = sigma(KS_lat, w);
[svW2, wW2] = sigma(1/W2_lat, w);
[svT, wT] = sigma(T_lat, w);
[svW3, wW3] = sigma(1/W3T_lat, w);

semilogx(w, 20*log10(svS(1,:)), 'b', 'LineWidth', 1.8); hold on;
semilogx(w, 20*log10(svW1(1,:)), 'b--', 'LineWidth', 1.5);
semilogx(w, 20*log10(svKS(1,:)), 'r', 'LineWidth', 1.8);
semilogx(w, 20*log10(svW2(1,:)), 'r--', 'LineWidth', 1.5);
semilogx(w, 20*log10(svT(1,:)), 'm', 'LineWidth', 1.8);
semilogx(w, 20*log10(svW3(1,:)), 'm--', 'LineWidth', 1.5);
grid on;
ylim([-80, 50]);
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
title('Valores Singulares y Cotas - Eje Lateral (\phi)');
legend('\sigma(S)', '\gamma / |W_1|', '\sigma(KS)', '\gamma / |W_2|', '\sigma(T)', '\gamma / |W_3|', 'Location', 'SouthWest');
exportgraphics(f5, [out_dir, 'hinf_roll_sigma.png'], 'Resolution', 300);
close(f5);

% Figura Paso Lateral
f6 = figure('Visible', 'off', 'Position', [100, 100, 600, 420]);
[y, t] = step(T_lat, 2.5);
plot(t, y, 'b', 'LineWidth', 1.8); hold on;
yline(1, 'k--');
grid on;
xlabel('Tiempo [s]');
ylabel('Respuesta \phi(t) [rad]');
title('Respuesta al Escalón Unitario - Alabeo \phi');
exportgraphics(f6, [out_dir, 'hinf_roll_step.png'], 'Resolution', 300);
close(f6);

fprintf('¡Todas las figuras H-inf han sido generadas exitosamente en %s!\n', out_dir);
