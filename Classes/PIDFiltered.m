classdef PIDFiltered < BaseController 
    properties  (Access = protected)
        u
        u_1 % azione di controllo istante k-1
        u_2 % azione di controllo istante k-2
        Kp
        Ki
        Kd
        Tf 
        e_1 % errore istante k-1
        e_2 % errore istante k-2
        af
        r
        sat

    end
    methods
        function obj=PIDFiltered(st, Kp, Ki, Kd, Tf, umax, sat)
            obj@BaseController(st);
            obj.u = 0;
            obj.u_1 = 0;
            obj.u_2 = 0;
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.Tf = Tf;
            obj.e_1 = 0;
            obj.e_2 = 0;
            obj.r = NaN;
            obj.sat = sat;
            obj.umax = umax;
        end
        function inizialize(obj)
            obj.u = NaN;
            obj.r = NaN; 
            obj.u_1 = NaN;
            obj.u_2 = NaN;
            obj.e_1 = NaN;
            obj.e_2 = NaN;
        end
        function u=computeControlAction(obj,reference,y)
        
            if isnan(obj.r)
                obj.r = 0;
                obj.u = 0;
                obj.u_1 = 0;
                obj.u_2 = 0;
                obj.e_1 = 0;
                obj.e_2 = 0;
            else
                
                r_new = reference;
                obj.r = r_new;
            end
            e_y=obj.r-y;

            % controllore digitale con discretizzazione eulero
            % all'indietro, filtro a azione di controllo completa 
            [a0, a1, a2, b0, b1, b2] = discrete(obj.Kp, obj.Ki, obj.Kd, obj.Tf, obj.st);
            
            u = 1/b0*(a0*e_y + a1*obj.e_1 + a2*obj.e_2 - b1*obj.u_1 - b2*obj.u_2);
           
            if obj.sat 
                if (u > obj.umax)
                    usat = obj.umax;
                elseif (u < -obj.umax)
                    usat = -obj.umax;
                else
                    usat = u;
                end
                u = usat;
            end
            obj.u_2 = obj.u_1;
            obj.u_1 = u;
            obj.e_2 = obj.e_1;
            obj.e_1 = e_y;

        end
    end
end
