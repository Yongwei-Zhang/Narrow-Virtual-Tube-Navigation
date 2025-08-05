function A = cal_rho_d_A(radius,min_lIndex,max_lIndex,l)

    A = 0;
    for i = min_lIndex : (max_lIndex-1)
        delta_l = l(i+1) - l(i);
        delta_A = 2 * radius(i)^2 * delta_l;
        A = A + delta_A;
    end

end