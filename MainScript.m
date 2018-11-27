% The main script to simulate the trajectory of a three-arm manipulator.
% not accounting for friction or momentum.
% Final version completed in Octobebr 2017
% Written by Avinash  Soor
% Git: Avinasho
% Written for the MEng Individual Project

clear; close all; clc;
%%
L = [2, 2, 2]; % The length of each joint on the manipulator
t = 1; % Counter
figure(1)

%% benchmark data
for i = 0:0.0157:pi
    
    q = [i, i, (pi-2*i)];
    
    ref_theta(t) = sum(q);
    
    [ref_traj_x(t), ref_traj_y(t)] = find_full_traj(L, q, 3);
    
    [l1_x(t), l1_y(t)] = find_full_traj(L, q, 1);
    [l2_x(t), l2_y(t)] = find_full_traj(L, q, 2);
    [l3_x(t), l3_y(t)] = find_full_traj(L, q, 3);
    
%     subplot(2, 2, 1)
    plot(ref_traj_x, ref_traj_y, 'r*')
    title('Reference Data')
    line([l2_x(t); l3_x(t)],[l2_y(t); l3_y(t)]);
    axis([-5 3 -2 4])
    xlabel('X')
    ylabel('Y')
    set(gca, 'FontName', 'Cambria', 'FontSize', 17)
    
    drawnow;

    t = t + 1;
    
end
%% IK calculations
c_ang = [0.01; 0.01; 0.01];
c_tot_ang = sum(c_ang);

[cx_traj, cy_traj] = find_full_traj(L, c_ang, 3);

for i = 1:t-1
    %% analytical
    del_traj = [ref_traj_x(i) - cx_traj;
        ref_traj_y(i) - cy_traj;
        ref_theta(i) - c_tot_ang];
    
    Jac_a = IK_Jacobian_func(c_ang, L);
    inv_Jac_a = inv(Jac_a);
    del_ang = inv_Jac_a * del_traj;
    c_ang = c_ang + del_ang;
    c_tot_ang = sum(c_ang);
    
    [cx_traj, cy_traj] = find_full_traj(L, c_ang, 3);
    
    %% estimation
    if i == 1
        Jac_e = eye(3);
        del_traj_est = del_traj;
        del_ang_est = del_ang;
        c_ang_est = c_ang;
        c_ang_est_tot = sum(c_ang_est);
        px_traj_est = cx_traj;
        py_traj_est = cy_traj;
        p_ang_est = c_ang_est;
        p_ang_tot = c_tot_ang;
        [cx_traj_est, cy_traj_est] = find_full_traj(L, c_ang_est, 3);
    end
    
    del_traj_prev_est = [cx_traj_est - px_traj_est;
        cy_traj_est - py_traj_est;
        c_ang_est_tot - p_ang_tot];
    del_traj_est = [ref_traj_x(i) - cx_traj_est;%%%%%%%%%%%
        ref_traj_y(i) - cy_traj_est;
        ref_theta(i) - c_ang_est_tot];
    
    del_ang_prev_est = c_ang_est - p_ang_est;
    
    Jac_e = find_Jac_e(Jac_e, del_ang_est, del_traj_prev_est);
    del_ang_est = inv(Jac_e) * del_traj_est;%%%%%%%%%%%%
    p_ang_est = c_ang_est;
    p_ang_tot = c_ang_est_tot;
    c_ang_est = del_ang_est + c_ang_est;
    c_ang_est_tot = sum(c_ang_est);
    px_traj_est = cx_traj_est;
    py_traj_est = cy_traj_est;
    [cx_traj_est, cy_traj_est] = find_full_traj(L, c_ang_est, 3);
        
    %% plotting
    plot_x_traj(i) = cx_traj;
    plot_y_traj(i) = cy_traj;
    
    plot_x_traj_est(i) = cx_traj_est;
    plot_y_traj_est(i) = cy_traj_est;

    [l1x(i), l1y(i)] = find_full_traj(L, c_ang, 1);
    [l2x(i), l2y(i)] = find_full_traj(L, c_ang, 2);
    [l3x(i), l3y(i)] = find_full_traj(L, c_ang, 3);
    
    [l1x_est(i), l1y_est(i)] = find_full_traj(L, c_ang_est, 1);
    [l2x_est(i), l2y_est(i)] = find_full_traj(L, c_ang_est, 2);
    [l3x_est(i), l3y_est(i)] = find_full_traj(L, c_ang_est, 3);

    
%     subplot(2, 2, 2)
    subplot(1, 2, 1)
    plot(plot_x_traj,plot_y_traj,'g*')
    title('Analytical Calculation')
    line([0 l1x(i)],[0 l1y(i)],'LineWidth',4,'Color',[1 0 0])
    line([l1x(i); l2x(i)],[l1y(i); l2y(i)],'LineWidth',4,'Color',[0 1 0])
    line([l2x(i); l3x(i)],[l2y(i); l3y(i)],'LineWidth',4,'Color',[0 0 1])
    xlabel('X')
    ylabel('Y')
    set(gca, 'FontName', 'Cambria', 'FontSize', 17)
    axis([ -5 6 -4 4])
    drawnow;
    pause(0.01);
    
%     subplot(2, 2, [3, 4])
    subplot(1, 2, 2)
    plot(plot_x_traj_est, plot_y_traj_est,'b*')
    title('Estimation')
    line([0 l1x_est(i)],[0 l1y_est(i)],'LineWidth',4,'Color',[1 0 0])
    line([l1x_est(i); l2x_est(i)],[l1y_est(i); l2y_est(i)],'LineWidth',4,'Color',[0 1 0])
    line([l2x_est(i); l3x_est(i)],[l2y_est(i); l3y_est(i)],'LineWidth',4,'Color',[0 0 1])
    xlabel('X')
    ylabel('Y')
    set(gca, 'FontName', 'Cambria', 'FontSize', 17)
    axis([ -5 6 -4 4])
    drawnow;
    pause(0.01);
    
    
end
    
