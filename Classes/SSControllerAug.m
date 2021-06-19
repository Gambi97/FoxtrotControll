classdef SSControllerAug < BaseController 
    
    % SSController crea un controllore nello spazio degli stati per un
    % sistema con 5 stati (4 + attrito, stato aumentato su osservatore), 
    % retroazione degli stati e errore integrale.
    
    properties  (Access = protected)
        
        u       % azione di controllo
        xHat    % stato stimato
        xi      % stato aumentato (errore) stimato
        K       % vettore di retroazione dello stato
        H       % guadagno errore integrale
        af      % filtro sul set point
        rf      % riferimento filtrato 
        OBS     % oggetto osservatore
        
    end
    
    methods
        
        function obj = SSControllerAug(st, K, H, umax, L, A, B, C, Tf)
            
            obj@BaseController(st);
            obj.K = K;
            obj.H = H;
            obj.umax = umax;
            obj.af=exp(-obj.st/Tf);
            obj.OBS = SSObserverAug(L, A, B, C);
            
        end
        
        function initialize(obj)
            
            obj.OBS.initialize;
            obj.xHat = NaN;
            obj.xi= NaN;
            obj.u = NaN;
            obj.rf = NaN;
            
        end
        
        function u_next = computeControlAction(obj, reference, y)
            
            % assegno posizione (non serve velocità)
            y = y(1);

            % primo ciclo
            if isnan(obj.rf)
                obj.u = 0;
                obj.xHat = zeros(5, 1); % 5 stati osservatore (stato attrito)
                obj.rf = 0;
                obj.xi= 0;
                
            else
                 
                rf_new = obj.af*obj.rf+(1-obj.af)*reference;
                obj.rf = rf_new;
                
            end
            
            % calcolo stato stimato dall' osservatore
            x_next = obj.OBS.computeObserver(obj.u, y);
            
            % errore attuale
            e_y = obj.rf-y;
            
            % azione di controllo:
            xi_next = obj.xi + obj.st*e_y;
            u_next = obj.H*obj.xi - obj.K*obj.xHat(1:end-1);
            
            % saturazione azione di controllo, implemento anti-windup
            % azione integrale. Congelo azione integrale solo se favorisce
            % il wind-up (se u e e_y sono concordi)

            if u_next > obj.umax
                usat = obj.umax;
                if u_next*e_y >= 0
                    xi_next = obj.xi;
                end
            elseif u_next < -obj.umax
                usat = -obj.umax;
                if u_next*e_y >= 0
                    xi_next = obj.xi;
                end
            else
                usat = u_next;
            end
            u_next = usat;
    
            % aggiorno variabili
            obj.xi =  xi_next;
            obj.u = u_next;
            obj.xHat = x_next;
            
        end
    end
end
