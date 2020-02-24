clear, clc, close all

%% Leitura do Arquivo

my_file_path = '../text/myfile.txt';

fileID = fopen(my_file_path,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);

skip = 20;
states = 13;
t = A(1:skip*states:end);
x_vector = A(2:skip*states:end);
y_vector = A(3:skip*states:end);
theta_vector = A(4:skip*states:end);
vx_vector = A(5:skip*states:end);
vy_vector = A(6:skip*states:end);
V_vector = A(7:skip*states:end);
W_vector = A(8:skip*states:end);
c_x_vector = A(9:skip*states:end);
c_y_vector = A(10:skip*states:end);
r_x_vector = A(11:skip*states:end);
r_y_vector = A(12:skip*states:end);
a_vector = A(13:skip*states:end);
clear A skip states

%% Animação

figure('units','normalized','outerposition',[0 0 1 1])
hold on
grid on
xlabel('$ x (m) $', 'Interpreter', 'latex')
ylabel('$ y (m) $', 'Interpreter', 'latex')
zlabel('$ z (m) $', 'Interpreter', 'latex')


i = 1;

x = x_vector(i);
y = y_vector(i);
theta = theta_vector(i);
vx = vx_vector(i);
vy = vy_vector(i);

c_x = c_x_vector(i);
c_y = c_y_vector(i);
r_x = r_x_vector(i);
r_y = r_y_vector(i);
   
[X,Y,Z] = RaceTrack_curve(c_x, c_y, r_x, r_y);
[c, curva] = contour(X, Y, Z, [0 0], 'k-', 'LineWidth', 2); clear c;

pos = plot3(x,y,0, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
orient = quiver(x,y, cos(theta), sin(theta), 'b', 'LineWidth', 2, 'AutoScaleFactor', 0.5);
vel = quiver(x,y, vx, vy, 'g', 'LineWidth', 2, 'AutoScaleFactor', 0.5);
marcacao_t = text(0,0,15, 't =');
view(3)
axis equal
axis([-70 10 -10 10 -1 10])
view([10, 10, 10])

% disp('Pressione alguma tecla para iniciar a animação')
% pause()
legend('Curva', 'Posição', 'Orientação', 'Velocidade')

% [Gx Gy] = meshgrid(linspace(-70, 10, 20), linspace(-10, 10, 20));
% Gz = - ones(size(Gx));
% mesh(Gx, Gy, Gz)
% colormap copper
% clear Gx Gy Gz
TC_plot()

for i = 2:length(t)
   dt = t(i) - t(i-1);
   
   x = x_vector(i);
   y = y_vector(i);
   theta = theta_vector(i);
   vx = vx_vector(i);
   vy = vy_vector(i);
   c_x = c_x_vector(i);
   c_y = c_y_vector(i);
   r_x = r_x_vector(i);
   r_y = r_y_vector(i);
   
   [X,Y,Z] = RaceTrack_curve(c_x, c_y, r_x, r_y);
   set(orient, 'XData', x, 'YData', y, 'UData', 4*cos(theta), 'VData', 4*sin(theta))
   set(vel, 'XData', x, 'YData', y, 'UData', 8*vx, 'VData', 8*vy)
   set(curva, 'XData', X, 'YData', Y, 'ZData', Z)
   set(pos, 'XData', x, 'YData', y)
   set(marcacao_t, 'String', ['t = ', num2str(t(i))])
   
   pause(0.1*dt)
end

%% Gráficos

% r_y
figure()
subplot(2,1,1)
plot(t, r_y_vector, 'k-')
title('$ r_{y}(t)$','Interpreter', 'latex')
xlabel('$ t (s) $', 'Interpreter', 'latex')
ylabel('$ r_{y} (m) $', 'Interpreter', 'latex')
grid on

subplot(2,1,2)
plot(t, a_vector, 'm-')
title('$ dr_{y}(t)/dt$','Interpreter', 'latex')
xlabel('$ t (s) $', 'Interpreter', 'latex')
ylabel('$ a (m) $', 'Interpreter', 'latex')
grid on

%(x,y,theta)
figure()
subplot(3,1,1)
plot(t, x_vector, 'r-')
title('$ x(t)$','Interpreter', 'latex')
xlabel('$ t (s) $', 'Interpreter', 'latex')
ylabel('$ x (m) $', 'Interpreter', 'latex')
grid on

subplot(3,1,2)
plot(t, y_vector, 'g-')
title('$ y(t)$','Interpreter', 'latex')
xlabel('$ t (s) $', 'Interpreter', 'latex')
ylabel('$ y (m) $', 'Interpreter', 'latex')
grid on

subplot(3,1,3)
plot(t, unwrap(theta_vector), 'b-')
title('$ \theta(t)$','Interpreter', 'latex')
xlabel('$ t (s) $', 'Interpreter', 'latex')
ylabel('$ \theta (rad) $', 'Interpreter', 'latex')
grid on

%(V,W)
figure()
subplot(2,1,1)
plot(t, V_vector, 'k-')
title('$ V(t)$','Interpreter', 'latex')
xlabel('$ t (s) $', 'Interpreter', 'latex')
ylabel('$ V (m/s) $', 'Interpreter', 'latex')
grid on

subplot(2,1,2)
plot(t, W_vector, 'k-')
title('$ W(t)$','Interpreter', 'latex')
xlabel('$ t (s) $', 'Interpreter', 'latex')
ylabel('$ W (rad/s) $', 'Interpreter', 'latex')
grid on

%Trajeto
figure()
plot(x_vector,y_vector,'k-', x_vector(1), y_vector(1), 'ro')
legend('Trajeto', 'Posição Inicial')
xlabel('$ x (m) $', 'Interpreter', 'latex')
ylabel('$ y (m) $', 'Interpreter', 'latex')
axis equal
grid on
