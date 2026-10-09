
Anew = latmod.A
Bnew = latmod.B(:,1)
Cnew = latmod.C([3,5],:)
Dnew = latmod.D([3,5],1)

SysLong = ss(Anew,Bnew,Cnew,Dnew)
tfSysLong = tf(SysLong)

%------ Solo theta
% M1 = 3;
% A1 = 0.001;
% Wb1 = 37;
% 
% M3 = 0.02;
% A3 = 1;
% Wb3 = 2*pi*80;
% 
% Wp = 1*tf(1,1);
% W1 = eye(2)*tf([1/M1 Wb1],[1 A1*Wb1]);
% W2 = 60*tf(1,1);
% W3 = eye(2)*tf([1/M3 Wb3],[1 A3*Wb3]);
% Wn = 0.01;
%---------- Theta y q
M1T = 1.7;
A1T = 0.01;
Wb1T = 0.5;
% 
% M1q = 3;
% A1q = 0.001;
% Wb1q = 37;
% 
% W1q = tf([1/M1q Wb1q],[1 A1q*Wb1q]);
W1T = tf([1/M1T Wb1T],[1 A1T*Wb1T]);

W1 = [tf(1e-5),0;0,W1T];
%--- 
W2 = 2*tf(1,1);
%---
M3T = 0.2;
A3T = 1.4;
Wb3T = 2*pi*300;

% M3q = 0.02;
% A3q = 1;
% Wb3q = 2*pi*80;
% 
% W3q = tf([1/M3q Wb3q],[1 A3q*Wb3q]);
W3T = tf([1/M3T Wb3T],[1 A3T*Wb3T]);

W3 = [tf(1e-5),0;0,W3T];

% ------- Sys IC planta generalizada
% systemnames   = 'SysLong Wp Wn W1 W2 W3';
% inputvar      = '[ p; n ; r ; u ]';
% outputvar     = '[ W1; W2 ; W3;-SysLong - Wn + r ]';
% input_to_SysLong = '[-Wp + u]';
% input_to_Wn   = '[n]';
% input_to_W1   = '[-SysLong - Wn + r]';
% input_to_W2   = '[u]';
% input_to_W3   = '[SysLong]';
% input_to_Wp   = '[p]';
% 
% sysnom_ic = sysic;

augmented = augw(tfSysLong,W1,W2,W3)
ncont = 1;
nmeas = 2;
% [K1, sys_CL,gam1,info] = hinfsyn(sysnom_ic,nmeas,ncont);
[K1, sys_CL,gam1,info] = hinfsyn(augmented,nmeas,ncont);   
K = tf(K1);
% L = K*tfSysLong(2);
% 
% T = L/(1+L);
% S = 1/(1+L);
Sth = minreal(sys_CL(2,2)/W1T);   % r_θ -> e_θ   (sensibilidad)
KSr = minreal(sys_CL(3,2)/W2);    % r_θ -> u     (esfuerzo de control)
Tth = minreal(sys_CL(5,2)/W3T);   % r_θ -> θ     (sensibilidad complementaria)

figure(1)
subplot(2,2,1), sigma(Sth, 1/W1T), grid on, title('S_\psi vs 1/W_1')
subplot(2,2,2), sigma(KSr, 1/W2),  grid on, title('KS vs 1/W_2')
subplot(2,2,3), sigma(Tth, 1/W3T), grid on, title('T_\psi vs 1/W_3')
subplot(2,2,4), step(Tth),         grid on, title('Escalón \psi')
% Para simulink:
[NumKH, DenKH] = tfdata(K, 'v') ;
[NumG6, DenG6] = tfdata(SysLong(2), 'v');

bandwidth(Tth)
DIRK = K1;
% figure(1)
% hold on
% subplot(2,2,1)
% sigma(S, 1/W1)
% hold on
% title('S vs 1/W_1'), grid on
% 
% subplot(2,2,2)
% sigma(K*S, 1/W2)
% hold on
% title('KS vs 1/W_2'), grid on
% 
% subplot(2,2,3)
% sigma(T, 1/W3)
% hold on
% title('T vs 1/W_3'), grid on
% 
% subplot(2,2,4)
% step(feedback(L, 1))
% hold on
% title('Respuesta al escalón'), grid on
% hold on