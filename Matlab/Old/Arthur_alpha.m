

xv = -2:0.1:2;
yv = -2:0.1:2;

[X,Y] = meshgrid(xv,yv);

ALPHA = X*0;

for i = 1:1:length(xv)
    x = xv(i);
    for j = 1:1:length(yv)
        y = yv(j);

        %Função original ("quadratica")
        alpha = (x^4+y^4)^0.25-1;
        
        %Função nova ("linear")
        alpha = (x^4+y^4)^0.25-1;

        ALPHA(j,i) = alpha;

    end
end


figure(1)
surf(X,Y,ALPHA)

figure(2)
contour(X,Y,ALPHA)
axis equal


