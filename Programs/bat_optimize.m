function [xo,fxo] = bat_optimize(fo,d,n,ix,iy,N)

fmin = 0;
fmax = 1;
alfa = 0.90;
gama = alfa;

t = 1;

x = (ix(2)-ix(1))*rand([n,d])+ix(1);
v = 0.1*(iy(2)-iy(1))*rand([n,d])+iy(1);
f = (fmax-fmin)*rand([n,1])+fmin;
A = ones([d,1]);
r0 = 1;
r = r0;

while t<N

    for j = 1:d
        vf(j) = fo(x(:,j));
    end
    
    [vfo,o] = min(vf);
    xo = x(:,o);
    beta = rand([d,1]);
    
    for i = 1:d
        f(i) = fmin+ (fmax-fmin)*beta(i);
        %v(:,i) = v(:,i) + (x(:,i)-xo)*f(i);
        %x(:,i) = x(:,i) + v(:,i);
        va = v(:,i) + (x(:,i)-xo)*f(i);
        xa = x(:,i) + v(:,i);
    end

    epsilon = 2*rand([d,1])-1;
    for j = 1:d
        if (epsilon(j) < r)
            x(:,j) = x(:,j) + epsilon(j)*mean(A);
        end
        
        vf(j) = fo(x(:,j));
        if(epsilon(j) < A(j) && vf(j)< vfo)
          v(:,j) = va;
          x(:,j) = xa;
          A(j) = alfa*A(j);
          r(j) = r0*(1-exp(-gama*t));
        end
    end
    
    t = t+1;    
end
[fxo,ixo] = min(vf);
xo = x(:,ixo);

