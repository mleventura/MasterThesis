fo = @michalewicz;
n = 2; % number of variables
d = 500; % number of candidate solutions (bats)
ix = [-4,4];
iy = [-4,4];
N = 40;

[xo,fxo] = bat_optimize(fo,d,n,ix,iy,N)
