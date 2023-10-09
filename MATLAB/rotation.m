clear all
close all

global vertices vertices_rot faces
vertices = [
    -1, -1, -1;  % 頂点1
     1, -1, -1;  % 頂点2
     1,  1, -1;  % 頂点3
    -1,  1, -1;  % 頂点4
    -1, -1,  1;  % 頂点5
     1, -1,  1;  % 頂点6
     1,  1,  1;  % 頂点7
    -1,  1,  1   % 頂点8
];
% vertices = vertices * 0.8;

faces = [
    1, 2, 3, 4;  % 底面
    5, 6, 7, 8;  % 上面
    1, 2, 6, 5;  % 側面1
    2, 3, 7, 6;  % 側面2
    3, 4, 8, 7;  % 側面3
    4, 1, 5, 8   % 側面4
];
R = [-2/sqrt(6) 1/sqrt(6) 1/sqrt(6);
    0 1/sqrt(2) -1/sqrt(2);
    -1/sqrt(3) -1/sqrt(3) -1/sqrt(3)];
R = R';

% R = [-2/sqrt(6) 1/sqrt(6) -1/sqrt(6);
%     0 1/sqrt(2) 1/sqrt(2);
%     -1/sqrt(3) -1/sqrt(3) 1/sqrt(3)];

% R = [ -1/sqrt(6)  -1/sqrt(3)  1/sqrt(2);
%  -1/sqrt(6)   1/sqrt(3) -1/sqrt(2);
%   2/sqrt(6)  -1/sqrt(3)   0];
% R = R';

beta = pi/2-acos(1/sqrt(3));
Ry = [cos(beta) 0 -sin(beta);
    0 1 0;
    sin(beta) 0 cos(beta)];

% alpha = -acos(1/sqrt(2));
alpha = -pi/4;
Rx = [1 0 0;
    0 cos(alpha) sin(alpha);
    0 -sin(alpha) cos(alpha)];

R = Rx*Ry;

global x y z x_ y_ z_
x = [1; 0; 0];
y = [0; 1; 0];
z = [0; 0; 1];
x_ = R' * x;
y_ = R' * y;
z_ = -R' * z;

vertices_rot = R' * vertices';

figure(1);
subplot(3, 4, [1,2,3,5,6,7,9,10,11])
plot_set(30, 1.5);
legend("$x$", "$y$", "$z$", "$x^\prime$", "$y^\prime$", "$z^\prime$",'Interpreter', 'latex');

subplot(3,4,4);
plot_set(20, 2.5);
view(0,90);

subplot(3,4,8);
plot_set(20, 2.5);
view(0,0);

subplot(3,4,12);
plot_set(20, 2.5);
view(90,0);

function plot_set(fontsize, linewidth)
global vertices vertices_rot faces
global x y z x_ y_ z_
quiver3(0, 0, 0, x(1), x(2), x(3),0, 'LineWidth', linewidth);
hold on;
quiver3(0, 0, 0, y(1), y(2), y(3),0, 'LineWidth', linewidth);
quiver3(0, 0, 0, z(1), z(2), z(3),0, 'LineWidth', linewidth);
quiver3(0, 0, 0, x_(1), x_(2), x_(3),0, 'LineWidth', linewidth);
quiver3(0, 0, 0, y_(1), y_(2), y_(3),0, 'LineWidth', linewidth);
quiver3(0, 0, 0, z_(1), z_(2), z_(3),0, 'LineWidth', linewidth);
% patch('Vertices', vertices, 'Faces', faces, 'Facecolor', 'none', 'Linewidth', 1.5);
patch('Vertices', vertices_rot', 'Faces', faces, 'Facecolor', 'none', 'Linewidth', 1.5);

grid on;
daspect([1 1 1]);
limit = 1.75;
xlim([-limit limit]);
ylim([-limit limit]);
zlim([-limit limit]);
xlabel('$x$', 'Interpreter', 'latex');
ylabel('$y$', 'Interpreter', 'latex');
zlabel('$z$', 'Interpreter', 'latex');
set(gca, "FontName", "Times New Roman", "FontSize", fontsize);
end