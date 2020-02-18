clear, close all, clc

x = linspace(-1, 1);
y = linspace(-2, 2);

[X, Y] = meshgrid(x, y);

t = length(x);
for i = 1:t
    for j = 1:t
        x = X(i, j); y = Y(i, j);
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

figure()
contour(X,Y,Z, [0 0], 'k')
xlabel('$ x (m) $', 'Interpreter', 'latex')
ylabel('$ y (m) $', 'Interpreter', 'latex')
zlabel('$ z (m) $', 'Interpreter', 'latex')
axis equal
grid on
axis([-1.1 1.1 -2.1 2.1])

figure()
mesh(X,Y,Z)
xlabel('$ x (m) $', 'Interpreter', 'latex')
ylabel('$ y (m) $', 'Interpreter', 'latex')
zlabel('$ z (m) $', 'Interpreter', 'latex')
axis equal
grid on
shading interp
colormap autumn
