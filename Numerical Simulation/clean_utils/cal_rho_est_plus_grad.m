function [rho_est, rho_est_grad] = cal_rho_est_plus_grad(position, N, h, i)

    rho_est = 0;
    for j = 1:N 
        rho_est = rho_est + 1/(2*pi*N*h^2) * exp( -((position{i}(1)-position{j}(1))^2 ...
            + (position{i}(2)-position{j}(2))^2)/(2*h^2) );
    end
    
    term1 = 0;
    term2 = 0;
    for j = 1:N 
        term1 = term1 + 1/(2*pi*N*h^2) * exp( -((position{i}(1)-position{j}(1))^2 ...
            + (position{i}(2)-position{j}(2))^2)/(2*h^2) ) * (-(position{i}(1)-position{j}(1))/h^2);
        term2 = term2 + 1/(2*pi*N*h^2) * exp( -((position{i}(1)-position{j}(1))^2 ...
            + (position{i}(2)-position{j}(2))^2)/(2*h^2) ) * (-(position{i}(2)-position{j}(2))/h^2);
    end
    rho_est_grad = [term1, term2];

end