classdef PIDController < BaseController
    
    % PIDController crea un controllore pid discretizzato con il metodo di
    % Eulero all'indietro. Filtro a azione di controllo completa.
    
    properties  (Access = protected)
        
        u   % azione di controllo attuale
        u_1 % azione di controllo istante k-1
        u_2 % azione di controllo istante k-2
        Kp  % guadagno proporzionale
        Ki  % guadagno integrale
        Kd  % guadagno derivativo
        Tf  % filtro primo ordine azione completa
        e_1 % errore istante k-1
        e_2 % errore istante k-2
        r   % riferimento
        sat % TRUE se il PID ha saturazione

    end
    
    methods
        
        function obj = PIDController(st, Kp, Ki, Kd, Tf, umax, sat)
            
            obj@BaseController(st);
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.Tf = Tf;
            obj.sat = sat;
            obj.umax = umax;
            
        end
        
        function inizialize(obj)
            
            obj.r = NaN; 
            obj.u = NaN;
            obj.u_1 = NaN;
            obj.u_2 = NaN;
            obj.e_1 = NaN;
            obj.e_2 = NaN;
            
        end
        
        function u = computeControlAction(obj,reference,y)
        
            % primo ciclo:
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
            
            % calcolo errore attuale
            e_y=obj.r-y;

            % estraggo coefficienti numeratore e denominatore del PID
            % filtrato discreto
            [a0, a1, a2, b0, b1, b2] = discrete(obj.Kp, obj.Ki, obj.Kd, obj.Tf, obj.st);
            
            % calcolo azione di controllo attuale
            u = 1/b0*(a0*e_y + a1*obj.e_1 + a2*obj.e_2 - b1*obj.u_1 - b2*obj.u_2);
           
            % se il controllore ha saturazione, limito u.
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
            
            % aggiorno azioni di controllo e errori precedenti
            obj.u_2 = obj.u_1;
            obj.u_1 = u;
            obj.e_2 = obj.e_1;
            obj.e_1 = e_y;

        end
    end
end
