function TC_plot()

color = [0.8 0.8 0.8];

x = [-1.7, -1.7, -1.7, -1.7]; y = [1.8, -2, -2, 1.8]; z = [0, 0, 3, 3];
fig = fill3(x, y, z, color);

x = [-1.7, -50, -50, -1.7]; y = [1.8, 1.8, 1.8, 1.8]; z = [0, 0, 3, 3];
fig = fill3(x, y, z, color);
%set(fig, 'FaceAlpha', 0.5);
x = [-1.7, -50, -50, -1.7]; y = [-2, -2, -2, -2]; z = [0, 0, 3, 3];
fig = fill3(x, y, z, color);
%set(fig, 'FaceAlpha', 0.5);

x = [-50, -50, -50, -50]; y = [-3, 2.75, 2.75, -3]; z = [0, 0, 3, 3];
fig = fill3(x, y, z, color);
%set(fig, 'FaceAlpha', 0.5);

x = [-50, -50, -1.7, -1.7]; y = [1.8, -2, -2, 1.8]; z = [3, 3, 3, 3];
fig = fill3(x, y, z, color);
% set(fig, 'FaceAlpha', 0.5);

x = [-50, -42.3, -50]; y = [2.75, 2.75, 2.75]; z = [0, 0, 3];
fig = fill3(x, y, z, color);
%set(fig, 'FaceAlpha', 0.5);
x = [-50, -42.3, -50]; y = [-3, -3, -3]; z = [0, 0, 3];
fig = fill3(x, y, z, color);
%set(fig, 'FaceAlpha', 0.5);

x = [-42.3, -42.3, -50, -50]; y = [-3, -2, -2, -3]; z = [0, 0, 3, 3];
fig = fill3(x, y, z, color);
%set(fig, 'FaceAlpha', 0.5);
x = [-42.3, -42.3, -50, -50]; y = [1.8, 2.75, 2.75, 1.8]; z = [0, 0, 3, 3];
fig = fill3(x, y, z, color);
%set(fig, 'FaceAlpha', 0.5);

clear fig color

end