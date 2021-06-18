classdef PIDController_Vel_Filtered < BaseController 
    properties  (Access = protected)
        u_1 % azione di controllo istante k-1
        u_2 % azione di controllo istante k-2
        Kp
        Ki
        Kd
        N 
        e_1 % errore istante k-1
        e_2 % errore istante k-2
        af
        rf

    end
    methods
        function obj=PIDController_Vel_Filtered(st, Tf, Kp, Ki, Kd, N)
            obj@BaseController(st);
            obj.u_1 = 0;
            obj.u_2 = 0;
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.N = N;
            obj.e_1 = 0;
            obj.e_2 = 0;
            obj.rf=nan;
            obj.af = exp(-obj.st/Tf);
        end
        function inizialize(obj)
            obj.umax = 50;
            obj.rf=nan; 
            obj.u_1 = 0;
            obj.u_2 = 0;
            obj.e_1 = 0;
            obj.e_2 = 0;
        end
        function u=computeControlAction(obj,reference,y)
            pos=y(1);
            vel=y(2);
            if isnan(obj.rf) % first cycle
                obj.rf=pos;
            else
                rf_new=obj.af*obj.rf+(1-obj.af)*reference;
                obj.rf=rf_new;
            end
            
            e_y=obj.rf-pos;
            
%             K1 = obj.Kp + obj.Ki*obj.st + obj.Kd/obj.st;
%             K2 = - obj.Kp - 2*obj.Kd/obj.st;
%             K3 = obj.Kd/obj.st;
%             u = obj.u_1 + K1*e_y + K2*obj.e_1 + K3*obj.e_2;
            % filtro derivativo 
            a0 = 1+obj.N*obj.st;
            a1 = -2-obj.N*obj.st;      
            a2 = 1;
            b0 = obj.Kp*(1+obj.N*obj.st)+obj.Ki*obj.st*(1+obj.N*obj.st)+obj.Kd*obj.N;
            b1 = -(obj.Kp*(2+obj.N*obj.st)+obj.Ki*obj.st+2*obj.Kd*obj.N);
            b2 = obj.Kp + obj.Kd*obj.N;
         
            u = - a1/a0*obj.u_1 - a2/a0*obj.u_2 + b0/a0*e_y + b1/a0*obj.e_1 + b2/a0*obj.e_2;
            obj.u_2 = obj.u_1;
            obj.u_1 = u;
            obj.e_2 = obj.e_1;
            obj.e_1 = e_y;
            
            if (u>obj.umax)
                usat=obj.umax;
            elseif (u<-obj.umax)
                usat=-obj.umax;
            else
                usat=u;
            end
            u=usat;
        end
    end
end
