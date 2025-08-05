function [rho_est] = cal_point_rho_est(position, N, h, position_point)

    rho_est = 0;
    for j = 1:N
        rho_est = rho_est + 1/(2*pi*N*h^2) * exp( -((position_point(1)-position{j}(1))^2 ...
            + (position_point(2)-position{j}(2))^2)/(2*h^2) );
    end

end