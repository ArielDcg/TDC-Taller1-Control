tfLong = tf(longmod)
Along = [longmod.A(4,4),longmod.A(4,3);longmod.A(3,4),longmod.A(3,3)] % xDOT = A * [Th;q]
Blong = [longmod.B(4,1);longmod.B(3,1)]
Clong = [longmod.C(4,4),longmod.C(4,3);longmod.C(3,4),longmod.C(3,3)]
Dlong = [longmod.D(4,1);longmod.D(3,1)]
SysLong = ss(Along,Blong,Clong,Dlong)
tfSysLong = tf(SysLong)

s = tf("s");
G2 = tfSysLong(2);
G5 = tfLong(3,1);
rlocus(G2)
rlocus(tfLong(3,1)) % De acá se concluye que D = 0.02 Es estable tanto en la de quito orden como la de segundo

% bode(0.02*tfSysLong(2))
% hold on
% bode(0.02*tfLong(3,1))

% Fun fact: Con D = 0.02 SOLO en el sistema de quinto orden son estables TODOS los
% polos con o sin integrador :O

% Diseño PI

D = 0.02;
GOLPI2 = G2/(s*(1+D*G2)) % Se definen las funciones de lazo cerrado de R a Theta punto junto con el integrador para posición
GOLPI5 = G5/(s*(1+D*G5)) 
rlocus(GOLPI5) % Observando el root locus, del lazo cerrado + integrador estamos analizando el lazo cerrado para iterar Kp
% Sin embargo se concluye que no existe un kp que haga estable el lazo
% cerrado en G2, pero en el de quinto orden si lo hay milagrosamente y es
% menor que 0.06

kp = 0.005;
bode(kp*GOLPI5) % Sin embargo haciendo pruebas con el bode al parecer a pesar de tener un margen de ganancia mayor que cero sigue siendo inestable el lazo cerrado
% % 
% GLOCUS2 = GOLPI2/(s*(1+kp*GOLPI2)) % Ni hablar de si se usa otro integrador, ahi si que es aún mas inestable
% rlocus(GLOCUS2)

%GLOCUS5 = GOLPI5/(s*(1+kp*GOLPI5))
%rlocus(GLOCUS5)