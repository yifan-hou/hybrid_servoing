p = 0.7;

k = 1/(1+exp(-25*(p-0.5)));
t = 1:80;
w = k.^t;


figure(1);clf(1);hold on;
plot(t, w)

axis([0, 80, 0, 1]);

% p = 0:0.01:1;
% k = 1./(1+exp(-25*(p-0.5)));
% 
% figure(1);clf(1);hold on;
% plot(p, k)
%  axis equal
% axis([0, 1, 0, 1]);

