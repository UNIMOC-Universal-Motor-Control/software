clear;

f = 16000;
la = 450e-6;
lb = 500e-6;
lambda = 2e-4;
rs = 240e-3;
N = f;
is = 2;

t = (1:f)/f;
ua = zeros(N,1);
ub = zeros(N,1);
ia = zeros(N,1);
ib = zeros(N,1);

ud = zeros(N,1);
uq = zeros(N,1);
id = zeros(N,1);
iq = zeros(N,1);
w = 2*pi*1000;

ud = ones(N,1)*(la+lb)/2*w*is;

phi = 0;
ia_int = 0;
ib_int = 0;

for i = 1:f
  phi = phi + w/f;
  
  s = sin(phi);
  c = cos(phi);

  ua(i) = c*ud(i) - s*uq(i);
  ub(i) = s*ud(i) + c*uq(i);  
  
  ia_int = ia_int + (ua(i) - ia_int*rs)/(f*la);
  ia(i) = ia_int;
  ib_int = ib_int + (ub(i) - ib_int*rs)/(f*lb); 
  ib(i) = ib_int;
  
  id(i) = c*ia(i) + s*ib(i);
  iq(i) = -s*ia(i) + c*ib(i);
endfor


spec_ia = fft(id(end-31:end));

stem(1:16,2/32*abs(spec_ia(1:16)));
