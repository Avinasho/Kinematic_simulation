% The main script to simulate the trajectory of a three-arm manipulator.
% Final version completed in Octobebr 2017
% Written by Avinash  Soor
% Git: Avinasho
% Written for the MEng Individual Project

function [point_x, point_y] = find_full_traj(L, angles, deg)

L1 = L(1);
L2 = L(2);
L3 = L(3);

if deg == 1
    point_x = L1 * cos(angles(1));
    point_y = L1 * sin(angles(1));
elseif deg == 2
    point_x = L1 * cos(angles(1)) + L2 * cos(angles(1)+angles(2));
    point_y = L1 * sin(angles(1)) + L2 * sin(angles(1)+angles(2));
elseif deg == 3
    point_x = L1 * cos(angles(1)) + L2 * cos(angles(1)+angles(2)) + L3 * cos(angles(1)+angles(2)+angles(3));
    point_y = L1 * sin(angles(1)) + L2 * sin(angles(1)+angles(2)) + L3 * sin(angles(1)+angles(2)+angles(3));
else
    error('SOMETHING WENT WRONG');
end