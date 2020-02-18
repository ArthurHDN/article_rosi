function [X,Y,Z] = RaceTrack_curve(c_x, c_y, r_x, r_y)

        x = linspace(-r_x + c_x, r_x + c_x);
        y = linspace(-r_y + c_y, r_y + c_y);
        [X, Y] = meshgrid(x, y);
        
        r_x = r_x/2;

        t = length(x);
        for i = 1:t
            for j = 1:t
                y = (X(i, j) - c_x)/r_x; x = (Y(i, j) - c_y)/r_y;
                if y <= 1 && y >= -1 && x >= 0
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
end