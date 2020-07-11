% smith predictor
n = 100000;

Ts = 1/16e3;
R = 100e-3;
L = 700e-6;

i = zeros(1, n);
u = zeros(size(i));

for i = 2:n
  i(i) = i(i-1) + (u(i) - R*i(i-1))/L*Ts; 
endfor
