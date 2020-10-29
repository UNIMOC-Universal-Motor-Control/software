x = [
[ 1.3, 5.064],
[ 2.5, 10.065],
[ 3.6, 15.1024],
[ 4.8, 20.1],
[ 6.0, 25.1028],
[ 7.2, 30.13],
[ 8.5, 35.281],
[ 9.8, 40.273]
];

x_mean = mean(x);
num = 0;
den = 0;

for i = 1:size(x,1)
  num = num + ((x(i,2) - x_mean(2))*(x(i,1) - x_mean(1)));
  den = den + ((x(i,2) - x_mean(2))^2);
endfor

b = num/den;
a = x_mean(1) - b*x_mean(2);

dt = 2*a/32;

duty = dt * 1/16000 *2;