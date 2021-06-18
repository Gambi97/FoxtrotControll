function cost=pivel_cost_function(x,P,wc_des)

% C=(Kp*s+Ki)/s;
Kp=x(1);
Ki=x(2);
C=tf([Kp Ki],[1 0]);

L=P*C;
margini=allmargin(L);

%J =(wc-wc_des)^2

if isempty(margini.PMFrequency)
    cost=100;
else
    cost=(wc_des-margini.PMFrequency(end))^2;
end