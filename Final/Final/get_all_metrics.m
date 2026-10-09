load('modelo_lin.mat');

fprintf('===================================================\n');
fprintf('           LONGITUDINAL H-INFINITY\n');
fprintf('===================================================\n');

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

bw_long = bandwidth(T_long);
sinfo_long = stepinfo(T_long);
normS_long = norm(S_long, Inf);
normKS_long = norm(KS_long, Inf);
normT_long = norm(T_long, Inf);

fprintf('Longitudinal: gamma = %.4f\n', gam_long);
fprintf('BW = %.4f rad/s (%.4f Hz)\n', bw_long, bw_long/(2*pi));
fprintf('Overshoot = %.2f %%\n', sinfo_long.Overshoot);
fprintf('SettlingTime = %.4f s\n', sinfo_long.SettlingTime);
fprintf('RiseTime = %.4f s\n', sinfo_long.RiseTime);
fprintf('||S||_inf = %.4f, ||KS||_inf = %.4f, ||T||_inf = %.4f\n', normS_long, normKS_long, normT_long);
fprintf('K order = %d\n', order(K_long));

fprintf('===================================================\n');
fprintf('           DIRECCIONAL / YAW H-INFINITY\n');
fprintf('===================================================\n');

Anew_dir = latmod.A;
Bnew_dir = latmod.B(:,1); % Let us check whether B(:,1) or B(:,2) was used
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

bw_dir = bandwidth(T_dir);
sinfo_dir = stepinfo(T_dir);
normS_dir = norm(S_dir, Inf);
normKS_dir = norm(KS_dir, Inf);
normT_dir = norm(T_dir, Inf);

fprintf('Direccional: gamma = %.4f\n', gam_dir);
fprintf('BW = %.4f rad/s (%.4f Hz)\n', bw_dir, bw_dir/(2*pi));
fprintf('Overshoot = %.2f %%\n', sinfo_dir.Overshoot);
fprintf('SettlingTime = %.4f s\n', sinfo_dir.SettlingTime);
fprintf('RiseTime = %.4f s\n', sinfo_dir.RiseTime);
fprintf('||S||_inf = %.4f, ||KS||_inf = %.4f, ||T||_inf = %.4f\n', normS_dir, normKS_dir, normT_dir);
fprintf('K order = %d\n', order(K_dir));

fprintf('===================================================\n');
fprintf('           LATERAL / ROLL H-INFINITY\n');
fprintf('===================================================\n');

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

bw_lat = bandwidth(T_lat);
sinfo_lat = stepinfo(T_lat);
normS_lat = norm(S_lat, Inf);
normKS_lat = norm(KS_lat, Inf);
normT_lat = norm(T_lat, Inf);

fprintf('Lateral: gamma = %.4f\n', gam_lat);
fprintf('BW = %.4f rad/s (%.4f Hz)\n', bw_lat, bw_lat/(2*pi));
fprintf('Overshoot = %.2f %%\n', sinfo_lat.Overshoot);
fprintf('SettlingTime = %.4f s\n', sinfo_lat.SettlingTime);
fprintf('RiseTime = %.4f s\n', sinfo_lat.RiseTime);
fprintf('||S||_inf = %.4f, ||KS||_inf = %.4f, ||T||_inf = %.4f\n', normS_lat, normKS_lat, normT_lat);
fprintf('K order = %d\n', order(K_lat));
