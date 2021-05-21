clearvars; close all; clc

addpath Classes\ P_code\ Functions\
%% carico sistema e inizializzo controllore
system=Foxtrot;
Ts=system.getSamplingPeriod;
nInput = system.getInputNumber; % 1 input (coppia motore)
nOutput  = system.getOutputNumber; % 2 output (posizione, velocità)
cs=ControlledSystemFoxtrot(system);
cs.initialize; 
u_max = system.getUMax;

%% SEGNALE PORTANTE
omega_portante = 0.05;
ampiezza_portante = u_max*0.1;
T_portante=2*pi/omega_portante;
t=(0:Ts:(2*T_portante))';
portante = ampiezza_portante*sin(omega_portante*t);

%% SEGNALE ECCITANTE (MULTISINUSOIDI)
omega =  logspace(log10(.1), log10(3100), 60)'; % fino a freq. di nyquist, pi/Ts circa 3100 rad/s
omega_picco = logspace(log10(200), log10(500), 60)'; % picco intorno a 350 rad/s, infittisco lì
omega_tot = sort([omega; omega_picco]);
omega_tot = round(omega_tot/omega_portante)*omega_portante;
omega_tot = unique(omega_tot);
multisine_signal = zeros(length(t),1);
for idx = 1:length(omega_tot)
    multisine_signal = multisine_signal + sin(omega_tot(idx)*t);
end
max_amplitude = u_max*0.2;
multisine_signal=multisine_signal/max(multisine_signal)*max_amplitude;

%% SOMMA SEGNALE PORTANTE ED ECCITANTE
control_action = portante + multisine_signal;
control_action_coef=fourierCoefficients(t,control_action,omega_portante,omega_tot);

%% SIMULAZIONE DEL SISTEMA
y=zeros(length(t), 2);
cs.initialize;
for idx=1:length(t)
    [y(idx, :), t(idx,1)]=cs.openloop(control_action(idx));
end

pos = y(:, 1);
vel = y(:, 2);

figure('Name', 'Risposta del sistema')
subplot(2,1,1)
plot(t,vel)
xlabel('time')
ylabel('output - velocità motore')
grid on

subplot(2,1,2)
plot(t, control_action)
xlabel('time')
ylabel('input - coppia motore')
grid on

%% IDENTIFICAZIONE 
vel_coef=fourierCoefficients(t,vel,omega_portante,omega_tot);
freq_resp=idfrd(vel_coef./control_action_coef,omega_tot,Ts); % Y(i*omega)/U(i*omega)
bode_opts = bodeoptions('cstprefs');
bode_opts.PhaseWrapping = 'on';
figure(2)
h = bodeplot(freq_resp,'--o', bode_opts);
[mag, phase] = bode(freq_resp);
mag = mag(:);
phase =  phase(:);

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
h=bodeplot(freq_resp, bode_opts);
hold on
showConfidence(h,3)
bode(modello_continuo,modello_discreto, bode_opts)
xlim([w0 w1])
hold off

legend('Identification','Modello continuo','Modello discreto')