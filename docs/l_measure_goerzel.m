clear;

function [amp, phase] = goertzel(x,n,freq, sample_freq)
  w = 2*pi*freq/sample_freq;
  
  cr = cos(w);
  ci = sin(w);
  
  coeff = 2*cr;

  s = zeros(3,1);

  for i = 1:n
    s(1) = x(i) + coeff * s(2) - s(3);
    s(3) = s(2);
    s(2) = s(1);
  endfor
    % run N+1
    s(1) = 0 + coeff * s(2) - s(3);
    s(3) = s(2);
    s(2) = s(1);
  
  r = s(2) - cr*s(3);
  i = ci * s(3);
  
  amp = sqrt(r^2 + i^2)/n;
  % freq 0 has no complex conjugate
  if freq/sample_freq >= 1/n
    amp = 2*amp;
  endif
  
  phase = atan2(i,r)*180/pi;
endfunction

f = 16000;
la = 450e-6;
lb = 500e-6;
lambda = 2e-4;
rs = 240e-3;
N = 5100;
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

uq = ones(N,1)*(la+lb)/2*w*is;

phi = 0;
ia_int = 0;
ib_int = 0;

for i = 1:N
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

n = 16;
[amp1, ~] = goertzel(id(end - (n-1):end),n, 0, 16000);
[amp2, ~] = goertzel(iq(end - (n-1):end),n, 0, 16000);
[amp3, ~] = goertzel(id(end - (n-1):end),n, 2000, 16000);

ld = uq(1)/(w*(sqrt(amp1^2 + amp2^2)+amp3));
lq = uq(1)/(w*(sqrt(amp1^2 + amp2^2)-amp3));