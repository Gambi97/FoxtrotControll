% model=ElasticSystemModel(Jm,Jc,k,h,hm)
%
% input:  {coppia motore (controllabile), coppia carico (disturbo)}
% output: {posizione motore, posizione carico, velocità motore, velocità
% carico}
% state:  {posizione motore, posizione carico, velocità motore, velocità
% carico}
%
% Jm    inerzia motore
% Jc    inerzia carica
% k     costante elastica trasmissione
% h     smorzamento trasmissione
% hm    attrivo viscoso motore
function model=ElasticSystemModel(Jm,Jc,k,h,hm)


A=[0 0 1 0;
    0 0 0 1;
    -k/Jm k/Jm -h/Jm-hm/Jm h/Jm;
    k/Jc -k/Jc h/Jc -h/Jc];
B=[0 0 ;
    0 0;
    1/Jm 0;
    0 1/Jc];
C=eye(4);
D=zeros(4,2);

model=ss(A,B,C,D);
model.InputName={'tau motore','tau carico'};
model.StateName={'Pos. motore','Po. carico','Vel. motore','Vel. carico'};
model.OutputName={'Pos. motore','Pos. carico','Vel. motore','Vel. carico'};

