

close all

%
figure;
subplot(311), plot(time, x_real); ylabel('x');
subplot(312), plot(time, y_real);ylabel('y');
subplot(313), plot(time, z_real);ylabel('z');
% set(gca,'xlim',[-0.5 3]);
% set(gca,'ylim',[-0.5 3]);


%
figure;
plot3(x_real,y_real,z_real);
set(gca,'xlim',[-2 3]);
set(gca,'ylim',[-2 3]);
set(gca,'zlim',[-0.5 3]);
xlabel('x');
ylabel('y');
zlabel('z');





