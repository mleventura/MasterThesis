function dr = car_model(t,~)

theta0 = 0;
b = 0.3;

vr = 2+0.4*cos(0.1*t);
vl = 1.5+0.3*sin(0.3*t);

theta = theta0 + (vr-vl)*t/b;

dr = [(vr+vl)*0.5*cos(theta);
      (vr+vl)*0.5*sin(theta)];