clear, clc, close all

my_file_path = '~/vrep_ws/src/article_rosi/text/myfile.txt';

fileID = fopen(my_file_path,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);

skip = 15;
states = 9;
t = A(1:skip*states:end);
x_vector = A(2:skip*states:end);
y_vector = A(3:skip*states:end);
theta_vector = A(4:skip*states:end);
c_x_vector = A(5:skip*states:end);
c_y_vector = A(6:skip*states:end);
r_x_vector = A(7:skip*states:end);
r_y_vector = A(8:skip*states:end);
a_vector = A(9:skip*states:end);
clear A skip states

figure()
hold on
grid on
xlabel('$ x (m) $', 'Interpreter', 'latex')
ylabel('$ y (m) $', 'Interpreter', 'latex')
zlabel('$ z (m) $', 'Interpreter', 'latex')


TC_plot()

i = 1;

x = x_vector(i);
y = y_vector(i);
theta = theta_vector(i);


c_x = c_x_vector(i);
c_y = c_y_vector(i);
r_x = r_x_vector(i);
r_y = r_y_vector(i);
   
[X,Y,Z] = RaceTrack_curve(c_x, c_y, r_x, r_y);
[c, curva] = contour(X, Y, Z, [0 0], 'k', 'LineWidth', 2); clear c;

pos = plot3(x,y,0, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
orient = quiver(x,y, cos(theta), sin(theta), 'm', 'LineWidth', 2, 'AutoScaleFactor', 0.5);
marcacao_t = text(0,0,15, 't =');
view(3)
axis equal
axis([-70 10 -10 10 -1 10])

disp('Pressione alguma tecla para iniciar a animação')
pause()

for i = 2:length(t)
   dt = t(i) - t(i-1);
   
   x = x_vector(i);
   y = y_vector(i);
   theta = theta_vector(i);
   c_x = c_x_vector(i);
   c_y = c_y_vector(i);
   r_x = r_x_vector(i);
   r_y = r_y_vector(i);
   
   [X,Y,Z] = RaceTrack_curve(c_x, c_y, r_x, r_y);
   set(orient, 'XData', x, 'YData', y, 'UData', 4*cos(theta), 'VData', 4*sin(theta))
   set(curva, 'XData', X, 'YData', Y, 'ZData', Z)
   set(pos, 'XData', x, 'YData', y)
   set(marcacao_t, 'String', ['t = ', num2str(t(i))])
   
   pause(0.1*dt)
end


figure()
subplot(2,1,1)
plot(t, r_y_vector, 'k-')
title('$ r_{y}(t)$','Interpreter', 'latex')
xlabel('$ t (s) $', 'Interpreter', 'latex')
ylabel('$ r_{y} (m) $', 'Interpreter', 'latex')
grid on

subplot(2,1,2)
plot(t, a_vector, 'b-')
title('$ dr_{y}(t)/dt$','Interpreter', 'latex')
xlabel('$ t (s) $', 'Interpreter', 'latex')
ylabel('$ a (m) $', 'Interpreter', 'latex')
grid on
