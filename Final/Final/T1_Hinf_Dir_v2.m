Anew = latmod.A;
Bnew = latmod.B(:,2);       % Timón de dirección (rudder)
Cnew = latmod.C([3,5],:);   % r (yaw rate) y psi (yaw)
Dnew = latmod.D([3,5],2);

SysLong = ss(Anew,Bnew,Cnew,Dnew);
tfSysLong = tf(SysLong);

%---------- Psi y r
M1T = 1.7;
A1T = 0.01;
Wb1T = 0.5;

W1T = tf([1/M1T Wb1T],[1 A1T*Wb1T]);

W1 = [tf(1e-5),0;0,W1T];

%--- W2 Washout (_/-\_)
s = tf('s');
w1 = 0.2;     % Cero 1: sube (+20 dB/dec)
w2 = 3.0;     % Polo 1: inicio de banda media
w3 = 30.0;    % Polo 2: fin de banda media (cae)
w4 = 2000.0;  % Cero 2: asíntota alta frecuencia (< DC)
k  = 0.01;

W2inv = k * ((s + w1)*(s + w4)) / ((s + w2)*(s + w3));
W2 = minreal(1/W2inv);
%---
M3T = 0.2;
A3T = 1.4;
Wb3T = 2*pi*300;

W3T = tf([1/M3T Wb3T],[1 A3T*Wb3T]);

W3 = [tf(1e-5),0;0,W3T];

augmented = augw(tfSysLong,W1,W2,W3);
ncont = 1;
nmeas = 2;

[K1, sys_CL,gam1,info] = hinfsyn(augmented,nmeas,ncont);   
K = tf(K1);

Sth = minreal(sys_CL(2,2)/W1T);   % r_psi -> e_psi   (sensibilidad)
KSr = minreal(sys_CL(3,2)/W2);    % r_psi -> u       (esfuerzo de control)
Tth = minreal(sys_CL(5,2)/W3T);   % r_psi -> psi     (sensibilidad complementaria)

figure(1)
subplot(2,2,1), sigma(Sth, 1/W1T), grid on, title('S_\psi vs 1/W_1')
subplot(2,2,2), sigma(KSr, 1/W2),  grid on, title('KS vs 1/W_2 (Washout)')
subplot(2,2,3), sigma(Tth, 1/W3T), grid on, title('T_\psi vs 1/W_3')
subplot(2,2,4), step(Tth),         grid on, title('Escalón \psi')

% Para simulink:
[NumKH, DenKH] = tfdata(K, 'v');
[NumG6, DenG6] = tfdata(SysLong(2), 'v');

bandwidth(Tth)
DIRK = K1;
