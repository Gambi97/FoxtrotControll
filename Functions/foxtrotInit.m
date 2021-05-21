function [system, cs] = foxtrotInit
system=Foxtrot;
% Ts=system.getSamplingPeriod; %1e-3 s
% inNum = system.getInputNumber;
% outNum  = system.getOutputNumber;
% controllo
cs=ControlledSystemFoxtrot(system);
cs.getSamplingPeriod;
cs.initialize; 
end

