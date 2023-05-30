function s = michalewicz(x)
%

d = 2;
m = 3;

s = 0;
d = length(x);
for i = 1:d
    s = s-sin(x(i))*(sin(i*x(i)*x(i)/pi))^(2*m);
end

