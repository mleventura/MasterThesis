function dr = car_model_t(u)

t = u(1);
theta0 = 0;
b = 0.3;

vr = u(2);
vl = u(3);

theta = theta0 + (vr-vl)*t/b;

dr = [(vr+vl)*0.5*cos(theta);...
      (vr+vl)*0.5*sin(theta)];