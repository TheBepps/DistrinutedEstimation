function [result] = runge_kutta_4(meas_val_old, meas_val_new, dt, init_val)
    
    k1 = meas_val_old;
    k2 = meas_val_old + 0.5 * k1 * dt;
    k3 = meas_val_old + 0.5 * k2 * dt;
    k4 = meas_val_new;
    
    result = init_val + (1/6) * (k1 + 2*k2 + 2*k3 + k4) * dt;

end