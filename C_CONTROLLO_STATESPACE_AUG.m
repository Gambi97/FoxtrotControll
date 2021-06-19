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

% parametri tunati
Tf = x(4);
Q_pos = x(1);
Q_int = x(2);
alpha = x(3);

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

Po = alpha*min(lambda)*[1; 0.99; 0.98; 0.97; 0.96]; %Una decade più veloci del polo dominante del controllore
disp(['Poli osservatore: ', num2str(Po')])
Po_d=exp(Ts*Po); 
L = place(Ao', Co', Po_d(1:end))';

%% 
ctrl = SSControllerAug(Ts, K, H, u_max, L, Ao, Bo, Co, Tf);
ctrl.initialize;
cs.setController(ctrl);

    
%% SCORE

[score,results]=cs.evalution;
IAE=max([results.IAE]);
OV=max([results.OV]);
CE=max([results.CE]);
settling_time=max([results.settling_time]);
fprintf('IAE=%f\tOV=%f\tCE=%f\tsettling_time=%f\tscore=%f\n',IAE,OV,CE,settling_time,score)
figure(100)
subplot(2, 1, 1)
hold on
title("posizione")
% plottiamo limiti di tempo di assestamento (max 3 secondi al 2%)
plot([0 10], [2.94 2.94], 'r', [0 10], [3.06 3.06], 'r')
plot([10 20], [1.04 1.04], 'r', [10 20], [0.96 0.96], 'r')
plot([3 3], [0 5], 'r', [13 13], [0 5], 'r')
axis([0 20 0 3.5]);
xlabel("tempo")
ylabel("posizione")
hold off
subplot(2, 1, 2)
title("azione di controllo")
axis([0 20 -55 55]);
xlabel("tempo")
ylabel("coppia")

if score > 34
    load handel.mat
    sound(y,Fs);
end
%%
t=(0:Ts:20)';

k = round(length(t)/2);
r = zeros(length(t),1);
r(1:k) = 3;
r(k+1:end) = 1;

cs.initialize
for idx=1:length(t)
    [y(idx,:), u(idx,1), t(idx,1)]=cs.step(r(idx));
end
pos = y(:,1);
vel = y(:,2);

% risposta del sistema senza errori di modello di evaluation (per
% velocizzare, poi si fa evaluation)
figure
plot(t, r, '--', t, pos)
axis([0 20 0 3.5]);
legend("riferimento", "risposta")
grid on

toc


