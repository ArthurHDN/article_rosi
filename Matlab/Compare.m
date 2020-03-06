clear, clc, close all


WF = load('./MatData/WF.mat');
WO = load('./MatData/WO.mat');



%(x,y,theta)
figure()
subplot(3,1,1)
plot(WF.t, WF.x_vector, 'r-', 'LineWidth', 1.5)
hold on
plot(WO.t, WO.x_vector, 'c-', 'LineWidth', 1.5)
title('$ x(t)$','Interpreter', 'latex')
xlabel('$ t (s) $', 'Interpreter', 'latex')
ylabel('$ x (m) $', 'Interpreter', 'latex')
grid on
legend('Est�tico', 'Variante')
subplot(3,1,2)
plot(WF.t, WF.y_vector, 'g-', 'LineWidth', 1.5)
hold on
plot(WO.t, WO.y_vector, 'm-', 'LineWidth', 1.5)
title('$ y(t)$','Interpreter', 'latex')
xlabel('$ t (s) $', 'Interpreter', 'latex')
ylabel('$ y (m) $', 'Interpreter', 'latex')
grid on
legend('Est�tico', 'Variante')
subplot(3,1,3)
plot(WF.t, unwrap(WF.theta_vector), 'b-', 'LineWidth', 1.5)
hold on
plot(WO.t, unwrap(WO.theta_vector), 'y-', 'LineWidth', 1.5)
title('$ \theta(t)$','Interpreter', 'latex')
xlabel('$ t (s) $', 'Interpreter', 'latex')
ylabel('$ \theta (rad) $', 'Interpreter', 'latex')
grid on
% mtit('Estados')
legend('Est�tico', 'Variante')

%Trajeto
figure()
subplot(2,1,1)
hold on
plot(WF.x_vector,WF.y_vector,'k-', WF.x_vector(1), WF.y_vector(1), 'ro')
r_y = min(WF.r_y_vector);
[X,Y,Z] = RaceTrack_curve(WF.c_x(1), WF.c_y(1), WF.r_x(1), r_y);
contour(X, Y, Z, [0 0], 'b--');
r_y = max(WF.r_y_vector);
[X,Y,Z] = RaceTrack_curve(WF.c_x(1), WF.c_y(1), WF.r_x(1), r_y);
contour(X, Y, Z, [0 0], 'g--');
l = legend('Trajeto', 'Posi\c{c}\~{a}o Inicial', '$r_{min}$', '$r_{max}$');
set(l, 'FontSize', 12, 'Interpreter', 'latex')
xlabel('$ x (m) $', 'Interpreter', 'latex')
ylabel('$ y (m) $', 'Interpreter', 'latex')
axis equal
axis([-60 10 -10 25])
grid on

% subplot(2,1,2)
hold on
plot(WO.x_vector,WO.y_vector,'k-', WO.x_vector(1), WO.y_vector(1), 'ro')
r_y = min(WO.r_y_vector);
[X,Y,Z] = RaceTrack_curve(WO.c_x(1), WO.c_y(1), WO.r_x(1), r_y);
contour(X, Y, Z, [0 0], 'b--');
r_y = max(WO.r_y_vector);
[X,Y,Z] = RaceTrack_curve(WO.c_x(1), WO.c_y(1), WO.r_x(1), r_y);
contour(X, Y, Z, [0 0], 'g--');
l = legend('Trajeto', 'Posi\c{c}\~{a}o Inicial', '$r_{min}$', '$r_{max}$');
set(l, 'FontSize', 12, 'Interpreter', 'latex')
xlabel('$ x (m) $', 'Interpreter', 'latex')
ylabel('$ y (m) $', 'Interpreter', 'latex')
axis equal
axis([-60 10 -10 25])
grid on

%V_ref
figure()
subplot(2,1,1)

WF.V_ref = WF.vx_vector + i*WF.vy_vector;
WO.V_ref = WO.vx_vector + i*WO.vy_vector;

plot(WF.t, 2*abs(WF.V_ref), 'r-', 'LineWidth', 1.5)
axis([0 350 0.9 1.1])
hold on
plot(WO.t, 2*abs(WO.V_ref), 'b-', 'LineWidth', 1.5)
xlabel('$ t$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$ \|v_{ref}\|$', 'Interpreter', 'latex', 'FontSize', 12)
grid on
l = legend('Est�tico', 'Variante');
set(l, 'FontSize', 15)
subplot(2,1,2)
plot(WF.t, unwrap(angle(WF.V_ref)), 'r-', 'LineWidth', 1.5)
hold on
plot(WO.t, unwrap(angle(WO.V_ref)), 'b-', 'LineWidth', 1.5)
xlabel('$ t $', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$ \angle v_{ref} $', 'Interpreter', 'latex', 'FontSize', 12)
grid on
l = legend('Est�tico', 'Variante');
set(l, 'Location', 'Southeast', 'FontSize', 15)
mtit('$ v_{ref}(t)$','Interpreter', 'latex')