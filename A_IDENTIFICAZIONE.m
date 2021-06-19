clearvars; close all; clc
addpath Classes\ P_code\ Functions\

%% carico sistema e inizializzo controllore
[system, cs, Ts, u_max] = foxtrotInit;
ni = system.getInputNumber;     % 1 ingresso (coppia)
no = system.getOutputNumber;    % 2 uscite (posizione, velocità)

%% SEGNALE PORTANTE
omega_portante = 0.05;          % rad/s
ampiezza_portante = u_max*0.1;
T_portante=2*pi/omega_portante; % periodo portante
t=(0:Ts:(2*T_portante))';
portante = ampiezza_portante*sin(omega_portante*t); % Asin(wt)

%% SEGNALE ECCITANTE (MULTISINUSOIDI)

omega =  logspace(log10(.1), log10(3100), 60)'; % da 0.1 rad/s x attrito statico fino a freq. di nyquist, pi/Ts circa 3100 rad/s
omega_picco = logspace(log10(200), log10(500), 60)'; % picco intorno a 350 rad/s, infittisco lì
omega_tot = sort([omega; omega_picco]);
omega_tot = round(omega_tot/omega_portante)*omega_portante; % arrotondo per avere le omega multiple della portante
omega_tot = unique(omega_tot); % rimuovo doppioni eventuali
multisine_signal = zeros(length(t),1);
for idx = 1:length(omega_tot)
    % somma di sinusoidi
    multisine_signal = multisine_signal + sin(omega_tot(idx)*t);
end
max_amplitude = u_max*0.2; 
multisine_signal = multisine_signal/max(multisine_signal)*max_amplitude;

%% SOMMA SEGNALE PORTANTE ED ECCITANTE
control_action = portante + multisine_signal;
control_action_coef=fourierCoefficients(t,control_action,omega_portante,omega_tot);

%% SIMULAZIONE DEL SISTEMA

y = zeros(length(t), 2);
cs.initialize;
for idx=1:length(t)
    [y(idx, :), t(idx,1)]=cs.openloop(control_action(idx));
end

pos = y(:, 1);
vel = y(:, 2);

figure('Name', 'Risposta del sistema')
plot(t,vel)
xlabel('Tempo [s]','FontSize', 15)
ylabel('output - velocità motore [rad/s]', 'FontSize', 15)
axis([0 250 -15 15 ])
grid on

figure('Name', 'Segnale eccitante')
plot(t, control_action)
hold on
plot(t, portante, 'r', 'LineWidth', 2)
hold off
xlabel('tempo [s]', 'FontSize', 15)
ylabel('input - coppia motore [Nm]','FontSize', 15)
axis([0 250 -11 11 ])
legend("Segnale eccitante", "Portante")
grid on

%% IDENTIFICAZIONE 

vel_coef = fourierCoefficients(t, vel, omega_portante, omega_tot);
freq_resp = idfrd(vel_coef./control_action_coef,omega_tot,Ts); % Y(i*omega)/U(i*omega)
bode_opts = bodeoptions('cstprefs');
bode_opts.PhaseWrapping = 'on';
bode_opts.XLabel.FontSize = 15;
bode_opts.YLabel.FontSize = 15;
figure('Name', 'Bode')
bodeplot(freq_resp,'--o', bode_opts);
grid on

%% stima

w0 = omega_tot(1);
w1 = omega_tot(end);

peso=ones(length(freq_resp.Frequency), 1);
peso(freq_resp.Frequency<w0) = 1e-5;
peso(freq_resp.Frequency>w1) = 1e-5;
peso(freq_resp.Frequency>200 & freq_resp.Frequency<500) = 100;
opts=ssestOptions('WeightingFilter', peso, 'EnforceStability', 1); 

modello_continuo = ssest(freq_resp,3,opts);
modello_discreto = ssest(freq_resp,3,'Ts',Ts,opts);

figure('Name', 'Modelli')
bode_opts = bodeoptions('cstprefs');
bode_opts.PhaseWrapping = 'on';
bode_opts.XLabel.FontSize = 15;
bode_opts.YLabel.FontSize = 15;
h=bodeplot(freq_resp, bode_opts);
hold on
showConfidence(h,3);
bode(modello_continuo,modello_discreto, bode_opts)
xlim([w0 w1])
hold off
grid on

legend('Identification','Modello continuo','Modello discreto')