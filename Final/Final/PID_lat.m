%pi + d del lateral
close all
A_ROLL = latmod.A;
B_ROLL = latmod.B(:,1);
C_ROLL = latmod.C([4,2],:);
D_ROLL = latmod.D([4,2],1);
SysLat_ROLL = ss(A_ROLL,B_ROLL,C_ROLL,D_ROLL);%La primera tf es de phi, la segunda es de p
tfSysLat_ROLL = tf(SysLat_ROLL)
s = tf("s");

figure(5)
rlocus(SysLat_ROLL(2))
[r,k] = rlocus(SysLat_ROLL(2));

%% 



Max_K_ROLL = 0; %Vamos a buscar dentro de los valores de rlocus el maximo valor de ganancia

for i=1:1:length(k)
    if all(real(r(:,i)) < 0) %Se evaluan que todos los polos sean estables
        Max_K_ROLL = k(i); % Guardamos el máximo K
    else  
        disp("La máxima ganancia es:" + Max_K_ROLL)
        real(r(:,i-1))
        break
    end
end
%%

%Busqueda en retroalimentación positiva
K_roll= 0;
[r,k] = rlocus(tfSysLat_ROLL(2));
for i=1:1:length(k)
    if all(imag(r(:,i)) == 0) 
        K_roll = k(i)
        break
    end
end

figure(6)
rlocus(-tfSysLat_ROLL(2))
%%

K_roll = 0.106 %Al final resulta mejor la negativa
SAS_TF_ROLL = (1/s) * feedback(tfSysLat_ROLL(2),K_roll);
damp(feedback(tfSysLat_ROLL(2),K_roll))



zero_ROLL = 0.0195;
PI_tf = tf([1,zero_ROLL],[1,0]);
%rlocus(SAS_TF_ROLL*PI_tf)
[r,k] = rlocus(SAS_TF_ROLL*PI_tf);

Max_K_ROLL = 0; %Vamos a buscar dentro de los valores de rlocus el maximo valor de ganancia

for i=1:1:length(k)
    if all(real(r(:,i)) <= 0) %Se evaluan que todos los polos sean estables
        Max_K_ROLL = k(i); % Guardamos el máximo K
    else  
        disp("La máxima ganancia es:" + Max_K_ROLL)
        break
    end
end
Kp_ROLL = 0.00284;
Ki_ROll = zero_ROLL*Kp_ROLL
LAT_SYS_CL = feedback(Kp_ROLL*SAS_TF_ROLL,1)

%bode(Kp_ROLL*LAT_SYS_OL)

%LAT_SYS_OL = Kp_ROLL*PI_tf*SAS_TF_ROLL
pole(LAT_SYS_CL)
step(LAT_SYS_CL)

%% Red new

phim = 60;
b = (1+sind(phim))/(1-sind(phim))
g1 = 10*log10(b)
wc = 1.14;
T2 = 1/(sqrt(b)*wc);
Lead = (1+b*T2*s)/(1+T2*s);
figure(20)
bode(Lead*Kp_ROLL*SAS_TF_ROLL)
hold on
%%
wc_req = 5;
[mag]=bode(LAT_SYS_OL,wc_req);
dec = -20*log10(mag)-9;
b = 10^(dec/10);
T2 = 1/(sqrt(b)*wc_req);
Lead = (1+b*T2*s)/(1+T2*s);
figure(20)
bode(Lead*LAT_SYS_OL)
%%
w=bandwidth(feedback(Lead*LAT_SYS_OL,1));
fprintf('Ancho de banda obtenido: %.2f rad/s (%.2f Hz)\n', w, w / (2*pi));
figure(22)
step(feedback(Lead*LAT_SYS_OL,1))
%%_______________________H_inf________________