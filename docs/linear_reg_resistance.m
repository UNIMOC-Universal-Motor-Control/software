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
  num = num + ((x(i,1) - x_mean(1))*(x(i,2) - x_mean(2)));
  den = den + ((x(i,1) - x_mean(1))^2);
endfor

b = num/den;
a = x_mean(2) - b*x_mean(1);

dt = -a/16*1/16000*1e6;