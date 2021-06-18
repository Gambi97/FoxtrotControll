classdef CascadeColtri < BaseController 
    properties  (Access = protected)

        PID1 % Controllore esterno
        PID2 % Controllore interno
        af
        rf % riferimento filtrato
        
    end
    methods
        function obj = CascadeColtri(st, PID1, PID2, Tf)
            
            obj@BaseController(st);
            obj.umax = 50;
            obj.PID1 = PID1;
            obj.PID2 = PID2;
            obj.af=exp(-obj.st/Tf);
            obj.rf = nan;
        end
        
        function initialize(obj)
            obj.umax = 50;
            obj.rf = nan; 
            obj.PID1.initialize;
            obj.PID2.initialize;          
        end
        
        function u = computeControlAction(obj, reference, y)
            
            pos = y(1);
            vel = y(2);
            
            if isnan(obj.rf) 
                obj.rf = pos; 
            else
                rf_new = obj.af*obj.rf+(1-obj.af)*reference;
                obj.rf = rf_new;
            end
            
            ref_vel = obj.PID1.computeControlAction(obj.rf, pos);
            
            u = obj.PID2.computeControlAction(ref_vel, vel);
            
            if (u > obj.umax)
                usat = obj.umax;
            elseif (u < -obj.umax)
                usat = -obj.umax;
            else
                usat = u;
            end
            u = usat;
        end
    end
end