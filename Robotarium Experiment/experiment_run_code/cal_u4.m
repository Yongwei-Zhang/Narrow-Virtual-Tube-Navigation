function [u4] = cal_u4(k4, rho_est_grad, rho_d_grad, rho_est, u1, sat_u23, k1)

    u123 = u1 + sat_u23; 
    initial_u4 = -k4 * (rho_est_grad - rho_d_grad)/rho_est; 
    u4_x = initial_u4(1);
    u4_y = initial_u4(2);
    u4 = cal_s_v(u4_x, u4_y, 0.04*k1);

end