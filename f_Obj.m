function J = f_Obj(x, cs, Ts)
ctrl = CascadeController(Ts, x(8), x(1), x(2), x(3), x(4), x(5), x(6), x(7));
cs.setController(ctrl);
score = cs.evalution;
J = (score - 32)^2;
end

