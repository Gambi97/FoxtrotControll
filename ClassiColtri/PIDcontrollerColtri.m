classdef PIDcontrollerColtri < BaseController 
    properties  (Access = protected)

        Kp % costante proporzionale controllore
        Ki % costante integrativa
        Kd % costante derivativa
        N 
        r % riferimento
        u
        u_1 % variabile di controllo della posizione nell'istante precedente
        u_2 % variabile di controllo della posizione nell'istante prima del precedente
        e_1 % errore di posizione nell'istante precedente
        e_2 % errore di posizione nell'istante prima del precedente
    end
    methods
        function obj = PIDcontrollerColtri(st, Kp, Ki, Kd, N)
            obj@BaseController(st); 
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.N = N; % filtro primo ordine digitale
            obj.r = nan;
            obj.e_1 = nan; % errore velocità all'istante tk-1
            obj.e_2 = nan; % errore velocità all'istante tk-2
            obj.u = nan;
            obj.u_1 = nan; % azione di controllo all'istante tk-1
            obj.u_2 = nan; % azione di controllo all'istante tk-2
        end
        
        function initialize(obj) %ricordatevi di inizializzare tutte le variabili
            obj.r = nan;
            obj.u = nan;
            obj.u_1 = nan;
            obj.u_2 = nan;
            obj.e_1 = nan;
            obj.e_2 = nan;
            

        end
        
        function u = computeControlAction(obj,reference, y)

            if isnan(obj.r) % first cycle
                obj.r = 0; % controllo in cascata con PID interno implementato 
                obj.e_1 = 0;
                obj.e_2 = 0;
                obj.u = 0;
                obj.u_1 = 0;
                obj.u_2 = 0;
            end  
           
            Ti = obj.Kp/obj.Ki;
            Td = obj.Kd/obj.Kp;
            Nid = obj.Kd*obj.N/obj.Kp;
            
            if Td == 0
                gamma = 0;
            else
                gamma = Td/((obj.st*Nid+Td));
            end
            e = reference - y;
            K0 = obj.Kp * gamma *(1 - obj.st/Ti + Nid);
            K1 = -obj.Kp * (1+ gamma - obj.st/Ti + 2*Nid*gamma);
            K2 = obj.Kp * (1 + Nid*gamma);

            u = + obj.u_1*(1 + gamma)...
                - gamma*obj.u_2 ...
                + K2*e ...
                + K1*obj.e_1 ...
                + K0*obj.e_2;
            
                obj.e_2 = obj.e_1;
                obj.e_1 = e;

                obj.u_2 = obj.u_1;
                obj.u_1 = obj.u;    
            obj.u = u;
        end
    end
end