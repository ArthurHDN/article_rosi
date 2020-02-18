% TC
%TC_lat = [-1.6023, -1.7023, -52.7087, -52.5587];
%TC_long = [1.7683, -2.0566, -3.5279, 2.7470];

TC_lat = [-1.7023, -1.7023, -42.3, -42.3, -52.7, -52.7, -42.3, -42.3];

TC_long = [1.7683, -2.0566, -2.0566, -3.5279, -3.5279, 2.7470, 2.7470, 1.7683];
TC_ext = fill(TC_lat, TC_long, 'b');


set(TC_ext, 'FaceAlpha', 0.5);
hold on
grid on
axis equal

% % estimativa quadrada
% x = [-1.7, -1.7, -52.7, -52.7];
% y = [2.8, -3.5, -3.5, 2.8];
% 
% TC_quad = fill(x, y, 'r');
% set(TC_quad, 'FaceAlpha', 0.5);


% curva
%  ((x + c_x)./r_x).^4 + ((y + c_y)./r_y).^4 - 1
c_x = -31;
c_y = -0.35;
r_x = 31;
r_y = 3.5;
x = linspace(-r_x + c_x, r_x + c_x);
y = linspace(-r_y + c_y, r_y + c_y);
[X, Y] = meshgrid(x, y);
Z = ((X - c_x)./r_x).^4 + ((Y - c_y)./r_y).^4 - 1;

curva = contour(X, Y, Z, [0 0], 'k', 'LineWidth', 2);

