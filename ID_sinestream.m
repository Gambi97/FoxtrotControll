clearvars; close all; clc

%% carico sistema e inizializzio controllore
system=Foxtrot;
Ts=system.getSamplingPeriod;
nInput = system.getInputNumber; % 1 input (coppia motore)
nOutput  = system.getOutputNumber; % 2 output (posizione, velocità)
cs=ControlledSystemFoxtrot(system);
cs.initialize; 

%% IDENTIFICAZIONE SINESTREAM
freqs = logspace(log10(3),log10(1500),50)';
amps = 3*ones(length(freqs),1);
ramp = 0*ones(length(freqs),1);
settle = 0*ones(length(freqs),1);
pds_val1 = 3*ones(length(freqs),1);

input = frest.Sinestream('Frequency',freqs,...
    'Amplitude',amps,...
    'RampPeriods',ramp,...
    'SettlingPeriods',settle,...
    'NumPeriods',pds_val1);
identification_signal=input.generateTimeseries;
figure
plot(identification_signal)

% Resample
t_ident=(0:Ts:identification_signal.Time(end))';
identification_signal=identification_signal.resample(t_ident);
control_action=identification_signal.Data;


y=zeros(length(t_ident), 2);
cs.initialize;
for idx=1:length(t_ident)
    y(idx, :)=cs.openloop(control_action(idx));
end
pos = y(:, 1);
vel = y(:, 2);

figure
subplot(3,1,1)
plot(t_ident,pos)
xlabel('time')
ylabel('posizione')
grid on

subplot(3,1,2)
plot(t_ident,vel)
xlabel('time')
ylabel('velocità')
grid on

subplot(3,1,3)
plot(t_ident,control_action)
xlabel('time')
ylabel('input')
grid on

% Definisco l'oggetto identification data (iddata)
identification=iddata(vel, control_action, Ts);
% Calcolo la risposta in frequenza con il comando e la plotto
freq_resp_ident = spafdr(identification);
figure
bode(freq_resp_ident)

% identifico con ordine 3 (1 polo, 2 zeri cc, 2 poli cc)
modello_continuo = ssest(freq_resp_ident,5);
zpk(modello_continuo)

bode(modello_continuo, freq_resp_ident)
legend('Modello continuo', 'Identificazione')