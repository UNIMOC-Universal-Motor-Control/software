N = 128;
Rp = 10e3;
R0 = 10e3;
B = 3950;
T0 = 25 + 273.15;
R8 = R0*exp(-B/T0);

i = 0:N-1;

r = Rp./(N./(i.+1) .- 1);
t = B./log(r./R8) - T0 + 25;
