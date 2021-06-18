function y = stepF(L, r)

y = step(feedback(L, 1));

end

