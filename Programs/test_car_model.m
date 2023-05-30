options = odeset('RelTol',1e-6);
[t,y] = ode45(@car_model,[0,50],[0;0],options);

plot(t,y)
figure
plot(y(:,1),y(:,2))

