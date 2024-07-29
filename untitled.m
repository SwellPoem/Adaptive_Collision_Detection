%% Numerical definition of dynamic parameters
g0=9.81; % acceleration of gravity [m/s^2]
N=3;  % number of joints
l = [0.5 0.5 0.4]'; % link lengths [m]
dc = [l(1)/2 l(2)/2 l(3)/2]'; % link CoMs (on local x axis) [m]
m = [15 10 5]'; % link masses [kg]
radius = [0.2 0.1 0.1]'; % radius [m] of cylinder links with uniform mass 
q = [0; 0; 0];
% [kg*m^2]
I(3,3,1) = (1/2)*m(1)*radius(1)^2;  % I1zz=(1/2)*m1*r1^2;
I(1,1,2) = (1/2)*m(2)*radius(2)^2;  % I2xx=(1/2)*m2*r2^2;
I(2,2,2) = (1/12)*m(2)*(3*radius(2)^2+l(2)^2);  % I2yy=(1/12)*m2*(3*r2^2+L2^2);
I(3,3,2) = I(2,2,2);    % I2zz=I2yy;
I(1,1,3) = (1/2)*m(3)*radius(3)^2;  % I3xx=(1/2)*m3*r3^2;
I(2,2,3) = (1/12)*m(3)*(3*radius(3)^2+l(3)^2);  % I3yy=(1/12)*m3*(3*r3^2+L3^2);
I(3,3,3) = I(2,2,3);    % I3zz=I3yy;

% matrix of link CoMs
pc = [0,             -l(2)+dc(2), -l(3)+dc(3);
      -l(1)+dc(1),   0,           0;
      0,             0,           0];

save('data.mat');