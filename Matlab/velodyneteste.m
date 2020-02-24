clear, clc, close all
my_file_path = '../text/velodynedata.txt';

fileID = fopen(my_file_path,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);

skip = 1;
x_vector = A(1:skip*3:end);
y_vector = A(2:skip*3:end);
z_vector = A(3:skip*3:end);
clear A skip

figure()
hold on
grid on
view([1,1,1])

for i = 1:length(x_vector)
    x = x_vector(i);
    y = y_vector(i);
    z = z_vector(i);
    plot3(x,y,z,'r.')
end