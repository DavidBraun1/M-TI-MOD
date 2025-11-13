m = 0.2;
l = 0.4;
c_r = 0.02;
g = 9.81;
u0 = 10;

J = m * l^2;

k1 = (m*l)/(m*l^2+J);
k2 = (m*g*l)/(m*l^2+J);
k3 = (c_r)/(m*l^2+J);

%sim("A1c.slx")

%[A, b, c, d] = linmod("A1c");