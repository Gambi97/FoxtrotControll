classdef CascadeController < BaseController 
    
    % CascadeController crea un controllore in cascata che gestisce
    % nell'anello interno la velocità e in quello esterno la posizione.
    
    properties
        af  % coefficiente filtro sul riferimento
        rf  % riferimento filtrato
        C1 % controllore esterno (posizione)
        C2 % controllore interno (velocità)
    end
    
    methods
        
        function obj = CascadeController(st, Tf, Kp1, Kp2, Ki1, Ki2, Kd1, Kd2, Tf1, Tf2, umax)
            obj@BaseController(st);
            obj.umax = umax;
            obj.C1 = PIDFiltered(st, Kp1, Ki1, Kd1, Tf1, umax, false);
            obj.C2 = PIDFiltered(st, Kp2, Ki2, Kd2, Tf2, umax, true);
            obj.af=exp(-obj.st/Tf); % polo filtro primo ordine digitale
        end
        
        function initialize(obj) 
            obj.rf = nan;
            obj.C1.inizialize;
            obj.C2.inizialize;
        end
        
        function u = computeControlAction(obj, reference, y)
            
            % assegno posizione e velocità
            pos=y(1);
            vel=y(2);
            
            % primo ciclo:
            if isnan(obj.rf) 
                obj.rf = pos;
            else
                rf_new = obj.af*obj.rf + (1-obj.af)*reference;
                obj.rf = rf_new;
            end
            
            % calcolo azione di controllo dell'anello interno
            u_C1 = obj.C1.computeControlAction(obj.rf, pos);
            r_vel = u_C1;
            
            % calcolo azione di controllo del sistema
            u = obj.C2.computeControlAction(r_vel, vel);
     
        end
    end
end