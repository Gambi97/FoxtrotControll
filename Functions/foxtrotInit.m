function [system, cs, Ts, u_max] = foxtrotInit
system=Foxtrot;
Ts=system.getSamplingPeriod; %1e-3 s
u_max = system.getUMax;
cs=ControlledSystemFoxtrot(system);
cs.getSamplingPeriod;
cs.initialize; 
end

