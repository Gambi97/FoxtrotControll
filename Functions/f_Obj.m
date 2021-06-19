function J = f_Obj(x, cs, Ts, umax, Tf)
    
    % funzione obiettivo per ottimizzare il controllore in cascata.
    % x =  % Kp1, Kp2, Ki2, Kd1, Tf1, Tf2
    
    ctrl = CascadeController(Ts, Tf, x(1), x(2), 0, x(3), x(4), 0, x(5), x(6), umax);
    cs.setController(ctrl);
    score = cs.evalution;
    J = (score-40)^2;
end

