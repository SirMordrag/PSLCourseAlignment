function eul = rodriguez_rot_to_eul(a,b)
% source: https://www.mathworks.com/matlabcentral/answers/493771-euler-3d-rotation-between-two-vectors
% source: https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula

a = a./norm(a);
b = b./norm(b);

%%
th = acos(dot(a,b));        % angle between vectors
u = cross(a,b);             % vector of rotation
u = u/norm(u);
K = [0 -u(3) u(2)
     u(3) 0 -u(1)
    -u(2) u(1) 0];
R = @(t) eye(3) + sin(t)*K + (1-cos(t))*K^2;    % rotation matrix

axang = [u(1) u(2) u(3) th];
eul = rad2deg(quat2eul(axang2quat(axang), "XYZ"));
% eul = rotm2eul(cross());

x = [-1 1 nan 0 0];         % data to rotate
y = [0 0 nan -1 1];
z = x*0 + 5.2;
v = R(th)*[x;y;z];          % rotate data
[x1,y1,z1] = deal( v(1,:),v(2,:),v(3,:) );
[X,Y,Z] = sphere(20);
surf(5*X,5*Y,5*Z,'facecolor','none','edgecolor',[1 1 1]*0.8)
hold on
plot3(x,y,z)                % original data
plot3(x1,y1,z1,'r')         % rotated data
quiver3(0,0,0,6*a(1),6*a(2),6*a(3))
quiver3(0,0,0,6*b(1),6*b(2),6*b(3))
hold off
xlabel('X axis')
ylabel('Y axis')
axis equal

%%
    % rotation matrix Oy axis
Ry = [cos(th) 0 sin(th)
    0 1 0
    -sin(th) 0 cos(th)];
ph = atan2(b(2),b(1));      % angle to rotate about Oz axis
    % rotation matrix Oz axis
Rz = [cos(ph) -sin(ph) 0
    sin(ph) cos(ph) 0
    0 0 1];
v = Ry*[x;y;z];             % rotate about Oy
v = Rz*v;                 % rotate about Oz
[x2,y2,z2] = deal( v(1,:),v(2,:),v(3,:) );
hold on
plot3(x2,y2,z2)
hold off

view(45,45)


end