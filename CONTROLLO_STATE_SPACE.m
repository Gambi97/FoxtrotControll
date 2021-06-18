tic
clearvars;
close all; %clc

addpath Classes\ P_code\ Functions\
load modelli.mat
% load X
%% carico sistema e inizializzo controllore
[system, cs, Ts, u_max] = foxtrotInit;

%% modello continuo 
s = tf("s"); %integratore per ottenere la posizione
I = 1/s;
Ps = modello_continuo*I;
As = Ps.A;
Bs = Ps.B;
Cs = Ps.C;
Ds = Ps.D;

%% modello discreto
Iz = c2d(I, Ts); 
Pz = modello_discreto*Iz;
Az = Pz.A;
Bz = Pz.B;
Cz = Pz.C;
Dz = Pz.D;
R = ctrb(Pz);
rank(R);
O = obsv(Pz);
rank(O);

%% controllore + stato aggiunto (errore) 

n=order(Pz);
no=size(Cz,1);
ni=size(Bz,2);

Ahat_z=[Az zeros(n,ni); -Cz*Ts eye(no,no)];
Bhat_z=[Bz;zeros(ni,1)];
% poli controllore + sistema
lambda=[-500, -550, -560, -570, -580];
aval = exp(Ts*lambda);
Khat = place(Ahat_z, Bhat_z, aval);
K = Khat(:, 1:n);
H = - Khat(:, n+1:end);

P_contr = real(log(eig(Ahat-Bhat*Khat))/Ts);
%% osservatore
Ao = Az;
Bo = Bz;
Co = Cz;
Do = Dz;
% poli osservatore -> almeno 10 volte più veloci di quelli del controllore
o = exp(Ts*lambda*10);
L = place(Ao', Co', o(1:end-1))'; 
% ne metto uno in meno, non serve stimare lo stato errore.

%% 
ctrl = SSController(Ts, K, H, u_max, L, Az, Bz, Cz);
ctrl.initialize;
cs.setController(ctrl);

[score,results]=cs.evalution;
IAE=max([results.IAE]);
OV=max([results.OV]);
CE=max([results.CE]);
settling_time=max([results.settling_time]);
fprintf('IAE=%f\tOV=%f\tCE=%f\tsettling_time=%f\tscore=%f\n',IAE,OV,CE,settling_time,score)
figure(100)
subplot(2, 1, 1)
hold on
plot([0 10], [2.94 2.94], [0 10], [3.06 3.06])
plot([10 20], [1.02 1.02], [10 20], [0.98 0.98])
plot([3 3], [0 5], [13 13], [0 5])

toc
