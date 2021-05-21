clearvars;close all;clc

%% Sistema da controllare
system=Juliett;
Ts=system.getSamplingPeriod; %1e-3 s
system.getOutputNumber 
% il sistema ha due uscite. la prima è la posizione da controllore, la
% seconda è la velocità

system.getInputNumber% il sistema ha un ingresso: la coppia del motore
system.getUMax % massima coppia applicabile


% metodi che non dovrebbero servirvi
% system.initialize 
% (re)inizializza il sistema (è chiamata automaticamente in evalute). 
%
% system.setScenario(1) 
% ci sono due scenari: 
% se l'ingresso è uno il sistema è in modalità identificazione: niente
% disturbi dovuti all'applicazione
% se l'ingresso è >=2 il sistema lavora in modalità di controllo (quella
% usata in evalute) con disturbi e errori di modallazione

% system.updateState % aggiorna lo stato

%% Creo il sistema di controllo
cs=ControlledSystemJuliett(system);
cs.getSamplingPeriod
cs.initialize % inizializza sistema e controllore


%% Identificazione
% Consiglio: identificare il modello coppia->velocità, ottenere il modello
% coppia->posizione aggiungendo uno stato "integratore"

t=(0:Ts:1)';
u=sin(t);
cs.initialize
for idx=1:length(t)
    [y(idx,:),t(idx,1)]=cs.openloop(u(idx));
end
pos=y(:,1);
vel=y(:,2);


%% Creo controllore
Ki=0;
Kp=1;
Tf=0.1; % filtro sul setpoint

ctrl=PIController(Ts,Ki,Kp,Tf);
cs.setController(ctrl); % setto il controllore, il controllore riceve le due uscite
% alternativa: cs.setController(ctrl,1); % setto il controllore, il
% controllore riceve solo l'uscita 1



%% testo il controllo su un segnale a scelta
t=(0:Ts:1)';
r=ones(length(t),1);
cs.initialize
for idx=1:length(t)
    [y(idx,:),u(idx,1),t(idx,1)]=cs.step(r(idx));
end
pos=y(:,1);
vel=y(:,2);

%% valuto il sistema di controllo sullo scenario
%
% lo scenario è:
% cambio setpoint di posizone da 0 a 3 al tempo 0 secondi
% cambio setpoint di posizone da 3 a 1 al tempo 10 secondi
% ci sono disturbi a scalino e periodici
%
% indici di prestazione
% * IAE (integrated absolute error) calcolato escludendo i primi 0.8 secondi dal cambio di setpoint
% (considera solo la reieizione dei disturbi)
% * OV (overshoot) calcolato nei primi tre secondi dal cambio di setpoint: [0,3] e
% [10,13]
% * CE (control effort = sum(abs(diff(u))*Ts calcolato su tutta la
% simulazione
% * settling_time tempo di assestamento al 2% calcolato nei primi tre secondi dal cambio di setpoint: [0,3] e
% [10,13]
%
% baseline di contronto:
% OV<2, settling_time<0.8
% IAE, CE variano per ogni gruppo 
% 
% la valutazione è:
%
% calcolo penalità/bonus IAE rispetto alla baseline 
% penality_IAE=(IAE/obj.IAE_baseline-1);
%
% calcolo penalità/bonus CE  rispetto alla baseline
% penality_CE=(CE/obj.CE_baseline-1);
%
% score=30-5*min(penality_IAE,3)-10*min(penality_CE,3);
%
% penalizzo overshoot >2%
% if (OV>obj.OV_baseline)
%     score=score-0.5*(OV-obj.OV_baseline);
% end
%
% penalizzo settling_time > 0.8s
% if (settling_time>obj.settling_time_baseline)
%     score=score-5*min(settling_time/obj.settling_time_baseline-1,3);
% end

[score,results]=cs.evalution;
IAE=max([results.IAE]);
OV=max([results.OV]);
CE=max([results.CE]);
settling_time=max([results.settling_time]);
fprintf('IAE=%f\tOV=%f\tCE=%f\tsettling_time=%f\tscore=%f\n',IAE,OV,CE,settling_time,score)



% % Non dovrebbe servire: 
% %provo un singolo scenario
% scenario=1;
% [IAE,OV,CE,settling_time,result]=cs.imulation(scenario);
