function J = f_Obj(x, cs, Ts, umax)
ctrl = CascadeController(Ts, x(7), x(1), x(2), 0, x(3), x(4), 0, x(5), x(6), umax);

cs.setController(ctrl);
score = cs.evalution;
J = norm(score-40);
end

