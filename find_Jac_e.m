% The main script to simulate the trajectory of a three-arm manipulator.
% Final version completed in Octobebr 2017
% Written by Avinash  Soor
% Git: Avinasho
% Written for the MEng Individual Project

function Jac_estimation = find_Jac_e(Jac_e_prev, del_ang, del_traj)

kappa = 1.55;

Jac_estimation = (kappa*(((del_traj-(Jac_e_prev*del_ang))*del_ang')/(del_ang'*del_ang)))+Jac_e_prev;

