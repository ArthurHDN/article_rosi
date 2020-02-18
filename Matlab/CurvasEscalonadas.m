subplot(2,1,1)

clear
c_x = 0;
c_y = 0;
r_x = 1;
r_y = 1;
x = linspace(-r_x + c_x, r_x + c_x);
y = linspace(-r_y + c_y, r_y + c_y);
[X, Y] = meshgrid(x, y);
Z = ((X - c_x)./r_x).^4 + ((Y - c_y)./r_y).^4 - 1;

contour(X, Y, Z, [0 0],'k');
grid on
axis([-1.1 1.1 -1.1 1.1])
%axis equal

xlabel('$\xi$', 'Interpreter', 'latex')
ylabel('$\sigma$', 'Interpreter', 'latex')
title('Smooth Square', 'Interpreter', 'latex')


subplot(2,1,2)

c_x = 0;
c_y = 0;
r_x = 2;
r_y = 1;

x = linspace(-r_x + c_x, r_x + c_x);
        y = linspace(-r_y + c_y, r_y + c_y);
        [X, Y] = meshgrid(x, y);
        
        r_x = r_x/2;

        t = length(x);
        for i = 1:t
            for j = 1:t
                y = (X(i, j) - c_x)/r_x; x = (Y(i, j) - c_y)/r_y;
                if y <= 1 && y >= -1 && x > 0
                    z = x-1;
                elseif y < -1
                    z = (y+1)^2 + x^2 - 1;
                elseif y > 1
                    z = (y-1)^2 + x^2 - 1;
                elseif y <= 1 && y >= -1 && x < 0
                    z = -x-1;
                else
                    disp('nao calculado')
                end
                Z(i,j) = z;
            end
        end
        
        contour(X, Y, Z, [0 0], 'k');
grid on
axis([-2.1 2.1 -1.1 1.1])
%axis equal

xlabel('$\xi$', 'Interpreter', 'latex')
ylabel('$\sigma$', 'Interpreter', 'latex')
title('Race Track', 'Interpreter', 'latex')