function J = f_Obj_LQR(x, cs, Ts, u_max, Az, Bz, Cz, sys_hat)

    % funzione obiettivo per ottimizzare il controllo LQR nello spazio
    % degli stati.
    % x: Qy(1), Qy(2), alpha, Tf
    
    %% CONTROLLORE
    Qy = diag([x(1) x(2)]);
    % R = x(3);
    R=1e-6;
    Khat = lqry(sys_hat,Qy,R);
    K = Khat(:, 1:4);
    H = -Khat(:, 5:end);
    % poli controllore continuo
    lambda = real(log(eig(sys_hat.A-sys_hat.B*Khat))/Ts);
    
    %% OSSERVATORE
    % poli osservatore continuo
%     Po = x(3)*min(lambda)*[1; 1.01; 1.02; 1.03; 1.04];
    Po = x(3)*min(lambda)*[1; 0.99; 0.98; 0.97; 0.96];
    % poli osservatore discreto
    Po_d=exp(Ts*Po); 

    % Matrici osservatore aumentato
    Ao = [Az Bz; zeros(1,4) 1];
    Bo = [Bz; 0];
    Co = [Cz 0];

    L = place(Ao', Co', Po_d(1:end))';

    ctrl = SSControllerAug(Ts, K, H, u_max, L, Ao, Bo, Co, x(4));
    ctrl.initialize;
    cs.setController(ctrl);

    score = cs.evalution;
    J = (score-40)^2;

end

