function [Kp, Ki, Kd, Tf] = estrai_parametri(C)
    % ESTRAI_PARAMETRI estrae i parametri Kp Ki Kd Tf di un pid a partire 
    % da una fdt, se è presente un filtro si intende sull'intera u. 
    
    C = minreal(C);
    [num, den] = tfdata(C, 'v');
    Ki = 0;
    Kd = 0;
    Tf = 0;
    
    if length(den) == 1         % P non filtrato
        
        Kp = num(1);
        
    elseif length(den) == 2
        
            if num(1) == 0      % P filtrato
                
                Tf = 1/den(2);
                Kp = num(2)*Tf;
           
            elseif den(2) == 0  % PI non filtrato
                
                Kp = num(1);
                Ki = num(2);
            
            else                % PD filtrato
                
                Tf = 1/den(2);
                Kp = num(2)*Tf;
                Kd = num(1)*Tf;
                
            end
                
    else                        % PID filtrato  
        
        Tf = 1/den(2);
        Ki = num(3)*Tf;
        Kp = num(2)*Tf;
        Kd = num(1)*Tf;
        
    end
end

