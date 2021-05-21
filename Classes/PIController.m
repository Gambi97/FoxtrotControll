classdef PIController < BaseController 
    properties  (Access = protected)
        xi
        Kp
        Ki
        af
        rf
    end
    methods
        function obj=PIController(st,Ki,Kp,Tf)
            obj@BaseController(st);
            obj.xi=0;
            obj.Kp=Kp;
            obj.Ki=Ki;
            obj.af=exp(-obj.st/Tf); % polo filtro primo ordine digitale
            obj.rf=nan;
        end
        function inizialize(obj) %ricordatevi di inizializzare tutte le variabili
            obj.rf=nan; 
            % io uso nan per inizializzare una variabile alla prima 
            % iterazione di computeControlAction se il suo valore iniziale
            % dipende da grandezze di input
            obj.xi=0; %
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
            
            e_pos=obj.rf-pos;
            u=obj.xi+obj.Kp*e_pos;
            
            if (u>obj.umax)
                usat=obj.umax;
            elseif (u<-obj.umax)
                usat=-obj.umax;
            else
                usat=u;
            end
            obj.xi=obj.xi+obj.Ki*obj.st*e_pos+0.1*(usat-u);
            u=usat;
        end
    end
end