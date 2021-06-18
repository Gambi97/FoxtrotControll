classdef PIDcontrollerColtriVero < BaseController 
    properties  (Access = protected)

        K0 % costante proporzionale controllore
        Csat % Coppia massima consentita
        K1 % costante integrativa
        K2 % costante derivativa
        N % coefficiente del filtro discreto del primo ordine
        r % riferimento
        u
        u_old % variabile di controllo della posizione nell'istante precedente
        u_old2 % variabile di controllo della posizione nell'istante prima del precedente
        e % errore di posizione
        e_old % errore di posizione nell'istante precedente
        e_old2 % errore di posizione nell'istante prima del precedente
        HasSat % Se il controllore deve saturare

        gamma % coefficiente necessario al calcolo del segnale di controllo
    end
    methods
        function obj = PIDcontrollerColtriVero(st, K0, K1, K2, N, Csat, gamma, HasSat)
            obj@BaseController(st);
            
            obj.K0 = K0;
            obj.Csat = Csat;
            obj.K1 = K1;
            obj.K2 = K2;
            obj.N = N; % filtro primo ordine digitale
            obj.gamma = gamma;
            obj.r = nan;
            obj.e = nan;
            obj.e_old = nan; % errore velocità all'istante tk-1
            obj.e_old2 = nan; % errore velocità all'istante tk-2
            obj.u = nan;
            obj.u_old = nan; % azione di controllo all'istante tk-1
            obj.u_old2 = nan; % azione di controllo all'istante tk-2
            obj.HasSat = HasSat; % Saturazione
        end
        
        function initialize(obj) %ricordatevi di inizializzare tutte le variabili
            obj.r = nan;
            obj.u = nan;
            obj.u_old = nan;
            obj.u_old2 = nan;
            obj.e = nan;
            obj.e_old = nan;
            obj.e_old2 = nan;
           

        end
        
        function u = computeControlAction(obj,reference, y)
            
            
            if isnan(obj.r) % first cycle
                obj.r = 0; % controllo in cascata con PID interno implementato
                obj.u_old = 0; % con il velocity algorithm
                obj.e = 0;
                obj.e_old = 0;
                obj.e_old2 = 0;
                obj.u = 0;
                obj.u_old = 0;
                obj.u_old2 = 0;
            else
                % filtro digitale del primo ordine rf(k) = a*rf(k-1)+(1-a)*reference
                % obj.r  = obj.af * obj.r + (1-obj.af) * reference;  % posizione filtrata
               
            %  Aggiorno gli errori
                e_new = reference - y;
                obj.e_old2 = obj.e_old;
                obj.e_old = obj.e;
                obj.e = e_new;
            % Aggiorno le azioni di controllo passate
                obj.u_old2 = obj.u_old;
                obj.u_old = obj.u;  
            end  
            
            % Creo l'azione di controllo
            
            u = + obj.u_old*(1 + obj.gamma)...
                - obj.gamma*obj.u_old2 ...
                + obj.K2*obj.e ...
                + obj.K1*obj.e_old ...
                + obj.K0*obj.e_old2;
%             %             %  Aggiorno gli errori
%                 e_new = reference - y;
%                 obj.e_old2 = obj.e_old;
%                 obj.e_old = obj.e;
%                 obj.e = e_new;
%             % Aggiorno le azioni di controllo passate
%                 obj.u_old2 = obj.u_old;
%                 obj.u_old = obj.u;  
            if obj.HasSat
                if (u > obj.Csat)
                    u = obj.Csat;
                end
                if (u < -obj.Csat)
                   u = -obj.Csat;
                end
            end    
            obj.u = u;
        end
        
        function X = printParam(obj)
            X = ['K0: ', num2str(obj.K0),', K1: ', num2str(obj.K1),', K2: ', num2str(obj.K2)];
        end
    end
end