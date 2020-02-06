
x = 1:n-1;
n = size(x, 2);
y = 2048 + x*1;
y = floor(y + rand(1,n)*10);

x_sum = 0;
x2_sum = 0;
xy_sum = 0;
y_sum = 0;

for i = 1:n
  x_sum += x(i);
  x2_sum += x(i)*x(i);
  xy_sum += x(i)*y(i);
  y_sum += y(i);
endfor

sxy = xy_sum - (x_sum*y_sum)/n;
sxx = x2_sum - (x_sum*x_sum)/n;

acent = sxy/sxx;
mean = y_sum/n;


y1 = mean + acent*x;

plot (x,y,"g*", x,y1);
