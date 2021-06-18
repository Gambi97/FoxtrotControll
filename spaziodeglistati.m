tic
clearvars
close all
addpath Classes\ P_code\ Functions\
load modelli.mat

[system, cs, Ts, u_max] = foxtrotInit;

s = tf("s");
I = 1/s;
Iz = c2d(I, Ts); 
Pz = modello_discreto*Iz;
A = Pz.A;
B = Pz.B;
C = Pz.C;
D = Pz.D;

%% Controllo nello spazio degli stati
n = order(Pz);
no = size(C,1); %Numero di uscite
ni = size(B,2); %Numero di ingressi          

%% Taratura del controllore
% Imposto il sistema aumentato

%x =[x0 xd], stato più disturbo

Ahat = [ A    zeros(n,ni); ...
        -C*Ts eye(no,no)];
  
Bhat = [ B;   zeros(ni,1)];

Chat = [C 0; zeros(1,n) 1];

Dhat = zeros(no,ni);

sys_hat = ss(Ahat, Bhat, Chat, Dhat, Ts);

% Il primo termine di Qy rappresenta il peso sull' output,
% il secondo il peso sull'azione integrale
Qy = diag([4 400]);
Tf = 0.12;
coef = 2;
r = 1e-6; %Peso dell'azione di controllo
%R piccolo: uso l'attuatore quanto voglio
%R grande: uso l'attuatore di meno

disp(['parametri :',num2str(Qy(1,1)),' ',num2str(Qy(2,2)),' ',num2str(r)])
Khat = lqry(sys_hat, Qy, r);
K =  Khat(:,1:n);
H = -Khat(:,n+1:end);

P_contr = real(log(eig(Ahat-Bhat*Khat))/Ts);
disp(['Poli controllore: ', num2str(P_contr')])
%% Taratura dell' osservatore
% aumentato:
% Ao = [A B; zeros(1,4) 1];
% Bo = [B; 0];
% Co = [C 0];
% Do = D;

Po = coef*min(P_contr)*[1; 1.01; 1.02; 1.03; 1.04]; %Una decade più veloci del sistema controllato
% Po = [-60.053     -60.6536     -61.2541     -61.8546     -62.4552];
disp(['Poli osservatore: ', num2str(Po')])
% Po = [-50; -51; -52; -53; -54];
Po_d=exp(Ts*Po); 
L = place(A', C', Po_d(1:end-1))';

%% 
obj = SSController(Ts, K, H, u_max, L, A, B, C, Tf);
obj.initialize;
cs.setController(obj);

disp(['r:',num2str(r),', pos:', num2str(Qy(1,1)),', int:', num2str(Qy(2,2)),', Tf:', num2str(Tf)])
[score,results]=cs.evalution;
IAE=max([results.IAE]);
OV=max([results.OV]);
CE=max([results.CE]);
settling_time=max([results.settling_time]);
fprintf('IAE=%f\tOV=%f\tCE=%f\tsettling_time=%f\tscore=%f\n',IAE,OV,CE,settling_time,score)
toc

