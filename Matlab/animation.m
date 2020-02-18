clear, close all, clc

%TC
%TC_lat = [-1.6023, -1.7023, -52.7087, -52.5587];
%TC_long = [1.7683, -2.0566, -3.5279, 2.7470];

%% Plotando o TC
TC_lat = [-1.7, -1.7, -42.3, -42.3, -52.7, -52.7, -42.3, -42.3];
TC_long = [1.8, -2, -2, -2.9470, -2.9470, 2.7470, 2.7470, 1.8];

x = [-1.7, -1.7, -1.7, -1.7]; y = [1.8, -2, -2, 1.8]; z = [0, 0, 3, 3];
fig = fill3(x, y, z, 'b');
hold on
grid on
axis equal
%set(fig, 'FaceAlpha', 0.5);

x = [-1.7, -52.7, -52.7, -1.7]; y = [1.8, 1.8, 1.8, 1.8]; z = [0, 0, 3, 3];
fig = fill3(x, y, z, 'b');
%set(fig, 'FaceAlpha', 0.5);
x = [-1.7, -52.7, -52.7, -1.7]; y = [-2, -2, -2, -2]; z = [0, 0, 3, 3];
fig = fill3(x, y, z, 'b');
%set(fig, 'FaceAlpha', 0.5);

x = [-52.7, -52.7, -52.7, -52.7]; y = [-3, 2.75, 2.75, -3]; z = [0, 0, 3, 3];
fig = fill3(x, y, z, 'b');
%set(fig, 'FaceAlpha', 0.5);

x = [-52.7, -52.7, -1.7, -1.7]; y = [1.8, -2, -2, 1.8]; z = [3, 3, 3, 3];
fig = fill3(x, y, z, 'b');
% set(fig, 'FaceAlpha', 0.5);

x = [-52.7, -42.3, -52.7]; y = [2.75, 2.75, 2.75]; z = [0, 0, 3];
fig = fill3(x, y, z, 'b');
%set(fig, 'FaceAlpha', 0.5);
x = [-52.7, -42.3, -52.7]; y = [-3, -3, -3]; z = [0, 0, 3];
fig = fill3(x, y, z, 'b');
%set(fig, 'FaceAlpha', 0.5);

x = [-42.3, -42.3, -52.7, -52.7]; y = [-3, -2, -2, -3]; z = [0, 0, 3, 3];
fig = fill3(x, y, z, 'b');
%set(fig, 'FaceAlpha', 0.5);
x = [-42.3, -42.3, -52.7, -52.7]; y = [1.8, 2.75, 2.75, 1.8]; z = [0, 0, 3, 3];
fig = fill3(x, y, z, 'b');
%set(fig, 'FaceAlpha', 0.5);

clear fig

%% Curva
% Parametros
c_x = -31;
c_y = -0.1;
r_x = 31;
r_y = 3.5;
curve_type = 2;
offset = 0;

r_x = 33; c_x = -30;

switch curve_type
    %Smooth Square
    case 1
        
        x = linspace(-r_x + c_x, r_x + c_x);
        y = linspace(-r_y + c_y, r_y + c_y);
        [X, Y] = meshgrid(x, y);
        Z = ((X - c_x)./r_x).^4 + ((Y - c_y)./r_y).^4 - 1 + offset ;
        curva = contour3(X, Y, Z, [offset offset], 'k', 'LineWidth', 2); clear offset;
    
    %RaceTrack
    case 2
        
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
                Z(i,j) = z + offset;
            end
        end
        curva = contour3(X, Y, Z, [offset offset], 'k', 'LineWidth', 2); clear offset;
        
end


xlabel('$ x (m) $', 'Interpreter', 'latex')
ylabel('$ y (m) $', 'Interpreter', 'latex')
zlabel('$ z (m) $', 'Interpreter', 'latex')






























% Seguindo a curva

% b1 = 5*0.1*[-1 1 1 -1];
% b2 = 5*0.1*[1 1 -1 -1];
% 
% initial_pos = [b1;b2];
% 
% robot = fill(b1, b2, 'r');
% 
% new_pos = initial_pos;
% 
% frame = [0;0];
% 
% hold off
% 
% for t = 0:1000
%     
%    
%     pos_x = frame(1); pos_y = frame(2); 
%      
%     fi = ((pos_x - c_x)./r_x).^4 + ((pos_y - c_y)./r_y).^4 - 1;
% 	grad_fi = [(4./r_x)*((pos_x - c_x)./r_x).^3 ; (4./r_y)*((pos_y - c_y)./r_y).^3];
% 	Beta_fi = [ -(4./r_y)*((pos_y - c_y)./r_y).^3 ; (4./r_x)*((pos_x - c_x)./r_x).^3];
% 	G = -2/pi * atan(fi);
% 	H = sqrt(1 - G.^2);
% 	u = G*grad_fi(1) + H*Beta_fi(1);
% 	v = G*grad_fi(2) + H*Beta_fi(2);
%     u = u/norm(grad_fi);
%     v = v/norm(grad_fi);
%     
%     d = [u + randn(1)/10; v + randn(1)/10];
%     
%     new_pos = new_pos + [d d d d];
%     frame = frame + d;
%     set(robot, 'XData', new_pos(1, :), 'YData', new_pos(2, :));
% 
%     pause(.05)
% 
% end



