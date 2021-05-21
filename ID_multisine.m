clearvars; close all; clc

%% carico sistema e inizializzo controllore
system=Foxtrot;
Ts=system.getSamplingPeriod;
nInput = system.getInputNumber; % 1 input (coppia motore)
nOutput  = system.getOutputNumber; % 2 output (posizione, velocità)
cs=ControlledSystemFoxtrot(system);
cs.initialize; 

%% IDENTIFICAZIONE CON SEGNALE MULTISINE
% portante
omega_portante=0.05;
T_portante=2*pi/omega_portante;
t=(0:Ts:(1.1*T_portante))';
portante=1.5*sin(omega_portante*t);

% multisinusoidi
omega=logspace(log10(5),log10(3000),60)';
omega=round(omega/omega_portante)*omega_portante; % arrotondo per avere le omega multiple della portante
omega=unique(omega); %scarto eventuali doppioni
multisine_signal=zeros(length(t),1);
for idx=1:length(omega)
    multisine_signal=multisine_signal+sin(omega(idx)*t);
end
max_applitude=2;
multisine_signal=multisine_signal/max(multisine_signal)*max_applitude;
control_action=portante+ multisine_signal;
control_action_coef=fourierCoefficients(t,control_action,omega_portante,omega);

cs.initialize
for idx=1:length(t)
    [process_output(idx,:),t(idx,1)]=cs.openloop(control_action(idx));
end

figure(1)
subplot(2,1,1)
plot(t,process_output(:,2))
xlabel('time')
ylabel('output')
grid on

subplot(2,1,2)
plot(t,control_action)
xlabel('time')
ylabel('input')
grid on

process_output_coef=fourierCoefficients(t,process_output(:,2),omega_portante,omega);
freq_resp=idfrd(process_output_coef./control_action_coef,omega,Ts); % Y(i*omega)/U(i*omega)
bode_opts = bodeoptions('cstprefs');
bode_opts.PhaseWrapping = 'on';
figure(2)
h=bodeplot(freq_resp,'--o', bode_opts);

peso=ones(length(freq_resp.Frequency),1);
peso(freq_resp.Frequency<omega(1))=1e-5;
peso(freq_resp.Frequency>omega(end))=1e-5;
 
opts=ssestOptions('WeightingFilter',peso,'EnforceStability',1);
modello_continuo = ssest(freq_resp,3,opts);
modello_discreto = ssest(freq_resp,3,'Ts',Ts,opts);

figure(3)


h=bodeplot(freq_resp,'o', bode_opts);
showConfidence(h,3)
hold on
bode(modello_continuo,modello_discreto,freq_resp, bode_opts)
xlim([omega(1) omega(end)])

legend('Identification','Modello continuo','Modello discreto')