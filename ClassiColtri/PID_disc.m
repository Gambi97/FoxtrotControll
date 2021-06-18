function [gamma, K0, K1, K2] = PID_disc(Kp, Ki, Kd, N, Ts)
% Scrivo il controllo in forma ideale:

Ti = Kp/Ki;
Td = Kd/Kp;
N = Kd*N/Kp;

% Riscrivo il PID in forma discreta con la formula di back calculation:
% use the forward differencing approximation for the integral part
% and the backward differencing approximation for
% the derivative part. In this way the integral part
% can be computed in advance.
if Td == 0
    gamma = 0;
else
    gamma = Td/((Ts*N+Td));
end

K0 = +Kp * gamma *(1 - Ts/Ti + N);
K1 = -Kp * (1+ gamma - Ts/Ti + 2*N*gamma);
K2 = +Kp * (1 + N*gamma);

end

