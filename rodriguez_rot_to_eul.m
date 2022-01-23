function eul = rodriguez_rot_to_eul(a,b)
% source: https://www.mathworks.com/matlabcentral/answers/493771-euler-3d-rotation-between-two-vectors, shortened
% source: https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula

a = a./norm(a);
b = b./norm(b);

%%
th = acos(dot(a,b));        % angle between vectors
u = cross(a,b);             % vector of rotation
u = u/norm(u);

axang = [u(1) u(2) u(3) th];
eul = rad2deg(quat2eul(axang2quat(axang), "XYZ"));

end