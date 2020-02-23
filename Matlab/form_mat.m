clear, clc, close all

figure('units','normalized','outerposition',[0 0 1 1])
%title('','Interpreter','latex')
xlabel('$y$','Interpreter','latex')
ylabel('$x$','Interpreter','latex')

% ROBO
t = 0:0.1:2*pi;
x = 2*cos(t); y = 2*sin(t) - 2;%- ones(length(t));
fill(x,y,[0.8 0.8 0.8])
axis equal; axis([-10 10 -8 6]); axis off
hold on

% d_robo
plot([-2 2],[-2 -2], 'kx-', 'MarkerSize', 10, 'LineWidth', 2)
t = text(-0.8, -1.5,'$d_{robo}$','Interpreter','latex')
set(t, 'FontSize', 30)



t = -pi/4:0.1:1.25*pi;
x = cos(t); y = sin(t);

%R
plot([0 8*x(1)],[-2 8*y(1)-2], 'rx-', 'MarkerSize', 10, 'LineWidth', 2)
t = text(1.3, -4.3,'$R$','Interpreter','latex');
set(t, 'Color', [1 0 0])
set(t, 'FontSize', 30)

% f = fill([0 8*x], [0-2 8*y-2], [1 0 0]);
% set(f, 'FaceAlpha', 0.1, 'EdgeColor', 'none')
% Lazer
for i = 1:length(x)
    plot([x(i) 8*x(i)], [y(i)-2 8*y(i)-2], 'r--')
    
end

% t = 0:0.3:2*pi;
% x = 1.5*cos(t) + randn(1,length(t))/10 - 3; 
% y = 1.5*sin(t) + randn(1,length(t))/10 + 4;
% obstaculo
x = [-1.4966   -1.5601   -1.7254   -2.1561   -2.5975   -3.1577   -3.2863   -3.6495   -4.1505 -4.1182   -4.5491   -4.6123   -4.3597   -4.0783   -3.6672   -3.3041   -2.9451   -2.5853 -2.1066   -1.8843   -1.4874];
y = [   3.9896    4.2580    4.8835    5.0827    5.5331    5.5015    5.4837    5.2161    5.0655 4.6476    4.2333    3.7314    3.4006    2.7244    2.7092    2.6385    2.4867    2.6691 2.7813    3.1912    3.5225];
fill(x,y,[0 0 1])

%d_o
plot([-0.86 -2.257],[-0.18 2.746], 'kx-', 'MarkerSize', 10, 'LineWidth', 2)
t = text(-2.2, 1,'$d_o$','Interpreter','latex');
set(t, 'FontSize', 30)

%TC
fill([-6 -6 -10 -10], [-10 10 10 -10], [0 1 1]);
t = text(-9, 0,'TC','Interpreter','latex');
set(t, 'FontSize', 50)

plot(-1.497, 3.99, 'ko', 'MarkerSize', 3, 'LineWidth', 4);
plot(-3.51, 2.68, 'ko', 'MarkerSize', 3,'LineWidth', 4);
plot(-5.977, 3.317, 'ko', 'MarkerSize', 3,'LineWidth', 4);
plot(-6, -6.783, 'ko', 'MarkerSize', 3,'LineWidth', 4);
t = text(-1.3, 4,'$O_1$','Interpreter','latex')
set(t, 'FontSize', 30)
t = text(-3.5, 2.3,'$O_2$','Interpreter','latex')
set(t, 'FontSize', 30)
t = text(-5.8, 3.3,'$O_3$','Interpreter','latex')
set(t, 'FontSize', 30)
t = text(-5.8, -6.8,'$O_4$','Interpreter','latex')
set(t, 'FontSize', 30)





print('-djpeg', 'form_mat_fig')








hold off
