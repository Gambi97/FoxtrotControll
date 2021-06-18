tic
clearvars;
close all; %clc

addpath Classes\ P_code\ Functions\
load modelli.mat
load x_LQR_augmented
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

Ahat_z = [Az zeros(n,ni); -Cz*Ts eye(no,no)];
Bhat_z = [Bz; zeros(ni,1)];
Chat_z = [Cz 0; zeros(1,n) 1];
Dhat_z = zeros(no,ni);
sys_hat=ss(Ahat_z, Bhat_z, Chat_z, Dhat_z,Ts);
% poli controllore + sistema

% Qy=diag([x(1) x(2)]);
% R = x(3);



Tf = x(4);
Q_pos = x(1);
Q_int = x(2);
alpha = x(3);
% 
% 
% Tf = 0.1;
% Q_pos = 3;
% Q_int = 300;
% coef = 10;

Qy=diag([Q_pos Q_int]);
R = 1e-6;
Khat=lqry(sys_hat,Qy,R);
K=Khat(:,1:n);
H=-Khat(:,n+1:end);
lambda = real(log(eig(Ahat_z-Bhat_z*Khat))/Ts);
disp(['Poli controllore: ', num2str(lambda')])

%% osservatore
% aumentato:
Ao = [Az Bz; zeros(1,4) 1];
Bo = [Bz; 0];
Co = [Cz 0];
Do = Dz;

Po = alpha*max(lambda)*[1; 0.99; 0.98; 0.97; 0.96]; %Una decade più veloci del polo dominante del controllore
disp(['Poli osservatore: ', num2str(Po')])
Po_d=exp(Ts*Po); 
L = place(Ao', Co', Po_d(1:end))';

%% 
ctrl = SSControllerAug(Ts, K, H, u_max, L, Ao, Bo, Co, Tf);
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

if score > 34
    load handel.mat
    sound(y,Fs);
end

toc


