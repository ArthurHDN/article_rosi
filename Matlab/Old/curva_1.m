clear, clc, close all
c_x = 27;
c_y = 0.35;
r_x = 33;
r_y = 3.55;

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

axis([-65 10 -4 3.5])
axis equal

        
        

for t = 0:1000
    
   
     pos_x = frame(1); pos_y = frame(2); 
     
     fi = ((pos_x + c_x)./r_x).^4 + ((pos_y + c_y)./r_y).^4 - 1;
	grad_fi = [(4./r_x)*((pos_x + c_x)./r_x).^3 ; (4./r_y)*((pos_y + c_y)./r_y).^3];
	Beta_fi = [ -(4./r_y)*((pos_y + c_y)./r_y).^3 ; (4./r_x)*((pos_x + c_x)./r_x).^3];
	G = -2/pi * atan(fi);
	H = sqrt(1 - G.^2);
	u = G*grad_fi(1) + H*Beta_fi(1);
	v = G*grad_fi(2) + H*Beta_fi(2);
    u = u/norm(grad_fi);
    v = v/norm(grad_fi);
    
    d = [u ;v];
    
   
    
        
     
%      if norm([u;v]) ~= 0
%          d = [u;v]./(2*norm([u;v]));
%      else
%          d = [0;0]
%      end 

    new_pos = new_pos + [d d d d];
    frame = frame + d;
    set(robot, 'XData', new_pos(1, :), 'YData', new_pos(2, :));


    pause(.05)

end