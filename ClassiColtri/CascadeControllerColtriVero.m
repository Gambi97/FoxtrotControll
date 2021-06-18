classdef CascadeControllerColtriVero < BaseController 
    properties  (Access = protected)

        PID1 % Controllore esterno
        PID2 % Controllore interno
        af
        rf % riferimento filtrato
        
    end
    methods
        function obj = CascadeControllerColtriVero(st, PID1, PID2, Tf)
            
            obj@BaseController(st);   
            obj.PID1 = PID1;
            obj.PID2 = PID2;
            obj.af=exp(-obj.st/Tf);
            obj.rf = nan;
        end
        
        function initialize(obj) %ricordatevi di inizializzare tutte le variabili
           
            obj.rf = nan; 
            obj.PID1.initialize;
            obj.PID2.initialize;          
        end
        
        function u = computeControlAction(obj, reference, y)
            
            pos = y(1);
            vel = y(2);
            
            if isnan(obj.rf) % first cycle
                obj.rf = pos; % = 0;
            else
                rf_new = obj.af*obj.rf+(1-obj.af)*reference;
                obj.rf = rf_new;
            end
            
            ref_vel = obj.PID1.computeControlAction(obj.rf, pos);
            
            u = obj.PID2.computeControlAction(ref_vel, vel);
            
        end
        
        function X = printParam(obj)
            
            X1 = obj.PID1.printParam();
            X2 = obj.PID2.printParam();
            X = [X1, ' || ', X2];
        end
    end
end