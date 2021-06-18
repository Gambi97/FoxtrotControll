function [K0, K1, K2, gamma] = discreteParam(Ts, Kp, Ki, Kd, Np)
% forward integrale, backward derivativo
    Ti = Kp/Ki; %Ki = Kp/Ti
    Td = Kd/Kp; %Kd = Kp*Td
    Nid = Kd*Np/Kp; %Nid = Kd*N/Kp
    if Td == 0
        gamma = 0;
    else
        gamma = Td/(Nid*Ts+Td);
    end
    K0 = Kp*gamma*(1-Ts/Ti+Nid);
    K1 = -Kp*(1+gamma-Ts/Ti+2*Nid*gamma);
    K2 = Kp*(1+Nid*gamma);
end

