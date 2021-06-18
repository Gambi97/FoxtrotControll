function [a0, a1, a2, b0, b1, b2] = discrete(Kp, Ki, Kd, Tf, Ts)
    a0 = Kp*Ts + Ki*Ts^2 + Kd;
    a1 = - Kp*Ts - 2*Kd;
    a2 = Kd;
    
    b0 = Tf + Ts;
    b1 = - 2*Tf - Ts;
    b2 = Tf;
end

