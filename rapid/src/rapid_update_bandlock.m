function [band_locked, first_band_hit, A_basal, int_e] = rapid_update_bandlock( ...
    band_locked, first_band_hit, A_basal, A_hold, int_e, e_true, T4_des, CTRL)

    band_in  = CTRL.band_pct_in  * max(T4_des, 1e-8);
    band_out = CTRL.band_pct_out * max(T4_des, 1e-8);

    if ~band_locked && abs(e_true) <= band_in
        band_locked = true;
        first_band_hit = true;
        A_basal = A_hold;
        int_e = 0;
    elseif band_locked && abs(e_true) >= band_out
        band_locked = false;
    end
end
