function [u4] = cal_u4(k4, rho_est_grad, rho_d_grad, rho_est, u1, u2, u3)

    u123 = u1 + u2 + u3;
    initial_u4 = -k4 * (rho_est_grad - rho_d_grad)/rho_est;
    
    chosen_coef = linspace(1, 0, 21);
    for j = 1:size(chosen_coef,2)
        coef = chosen_coef(j);
        if norm(coef * initial_u4) <= norm(u123)
            u4 = coef * initial_u4;
            break;
        end
    end

end