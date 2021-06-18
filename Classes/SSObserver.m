classdef SSObserver < handle
    
    % SSObserver crea un osservatore dello stato asintotico, senza
    % modellizzare l'attrito.
    properties (Access = protected)
        
        L       % parametro osservatore Luenberger
        A       % matrice dello stato 
        B       % matrice degli ingressi
        C       % matrice delle uscite
        u_1     % ingresso istante k-1
        y_1     % uscita reale istante k-1
        xHat_1  % stato stimato istante k-1
        yHat_1  % uscita stimata istante k-1
        
    end
    
    methods
        
        function obj = SSObserver(L, A, B, C)
            
            obj.L = L;
            obj.A = A;
            obj.B = B;
            obj.C = C;
            
        end
        
        function initialize(obj)
            
            obj.u_1 = NaN;
            obj.y_1 = NaN; 
            obj.xHat_1 = NaN;
            obj.yHat_1 = NaN;
            
        end
        
        function xHat = computeObserver(obj, u, y)
            
            % primo ciclo
            if isnan(obj.u_1)
                
                obj.u_1 = 0;
                obj.y_1 = 0; 
                obj.xHat_1 = zeros(4, 1); % 4 stati osservatore
                obj.yHat_1 = 0;
                
            end

            % osservatore di Luenberger
            xHat = obj.B*u + obj.L*(y - obj.yHat_1) + obj.A*obj.xHat_1;
            yHat = obj.C*obj.xHat_1;
            
            % aggiorno dati precedenti
            obj.u_1 = u;
            obj.xHat_1 = xHat;
            obj.yHat_1 = yHat;
            
        end
    end
end

