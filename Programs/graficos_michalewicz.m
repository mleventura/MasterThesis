
x = -4:0.01:4;
y = -4:0.01:4;
nx = length(x);
ny = length(y);

[X,Y] = meshgrid(x,y);

for i = 1:nx
    for j = 1:ny
        z(i,j) = michalewicz([X(i,j),Y(i,j)]);
    end
end
mesh(X,Y,z)