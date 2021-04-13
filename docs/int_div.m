phi = mod(0:1e-3:10*pi, 2*pi) - pi;
phi1 = mod((0:1e-3:10*pi) .+ pi/6, 2*pi) - pi;
diff = zeros(size(phi));
int_phi = int32(zeros(size(phi)));
int_phi1 = int32(zeros(size(phi)));
t = 1:length(phi);

for i = 1:length(phi)
  int_phi(i) = int32(phi(i) .* double(intmax)/pi);
  int_phi1(i) = int32(phi1(i) .* double(intmax)/pi);
  diff(i) = int32(int_phi(i) + int_phi1(i));
endfor

plot(t,diff,t,int_phi);