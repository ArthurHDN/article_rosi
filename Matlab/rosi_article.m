%  ((x + c_x)./r_x).^4 + ((y + c_y)./r_y).^4 - 1

clear, clc, close all

x = linspace(-3, 3);
[X, Y] = meshgrid(x);
Z = X.^4 + Y.^4 - 1;

mesh(X, Y, Z)

xlabel('$ x (m) $', 'Interpreter', 'latex')
ylabel('$ y (m) $', 'Interpreter', 'latex')
zlabel('$ z (m) $', 'Interpreter', 'latex')

figure(2)
grid on

c_x = 1;
c_y = 1;
r_x = 1;
r_y = 3;
x = linspace(-r_x - c_x, r_x - c_x);
y_up =  r_y * (1 - ( (x+c_x)./r_x ) .^4 ) .^ (1/4) - c_y;
y_down =  -r_y * (1 - ( (x+c_x)./r_x ) .^4 ) .^ (1/4) - c_y;

b1 = 0.1*[-1 1 1 -1];
b2 = 0.1*[1 1 -1 -1];

curve(1) = plot(x, y_up, 'k');
grid on
hold on
curve(2) = plot(x, y_down, 'k');

initial_pos = [b1;b2];

robot = fill(b1, b2, 'r');

new_pos = initial_pos;

frame = [0;0];

hold off

axis([-10 10 -10 10])
for t = 0:1000
    
   
     pos_x = frame(1); pos_y = frame(2); 
     
     fi = ((pos_x + c_x)./r_x).^4 + ((pos_y + c_y)./r_y).^4 - 1;
     grad_fi = [ (4./r_x) * ((pos_x + c_x)./r_x).^3;
                 (4./r_y) * ((pos_y + c_y)./r_y).^3];
             
     Beta_fi = [ - (4./r_y) * ((pos_y + c_y)./r_y).^3;
                 (4./r_x) * ((pos_x + c_x)./r_x).^3];
             
     G = -2/pi * atan(fi);
     H = sqrt(1 - G.^2);
     
     M = eye(2);
     a = [(2*sin(t/10)*(c_x + pos_x)^4)/(5*(cos(t/10) - 2)^5) + (2*sin(t/10)*(c_y + pos_y)^4)/(5*(cos(t/10) - 2)^5);
         0];
     
%     syms x y t c_x c_y default_r_x default_r_y a b
%     r_y = default_r_y + b*t;
%     r_x = default_r_x + a*t;
%     fi = ((x + c_x)./r_x).^4 + ((y + c_y)./r_y).^4 - 1;
%     diff(fi, t)
     
     P = -inv(M) * a;
     
     u = G*grad_fi(1) + H*Beta_fi(1) + P(1);
     v = G*grad_fi(2) + H*Beta_fi(2) + P(2);
     
     if norm([u;v]) ~= 0
         d = [u;v]./(2*norm([u;v]));
     else
         d = [0;0]
     end 

    new_pos = new_pos + d;
    frame = frame + d;
    set(robot, 'XData', new_pos(1, :), 'YData', new_pos(2, :));
    
    r_y = 2 - cos(t/10);
    r_x = 2 - cos(t/10);

    x = linspace(-r_x - c_x, r_x - c_x);

    y_up =  r_y * (1 - ( (x+c_x)./r_x ) .^ 4) .^ (1/4) - c_y;
    y_down = -r_y * (1 - ( (x+c_x)./r_x ) .^ 4) .^ (1/4) - c_y;

    set(curve(1), 'XData', x, 'YData', real(y_up));
    set(curve(2), 'XData', x, 'YData', real(y_down));

    pause(.05)

end

