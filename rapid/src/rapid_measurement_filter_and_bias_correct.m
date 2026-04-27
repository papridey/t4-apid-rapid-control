function [y_used, st] = rapid_measurement_filter_and_bias_correct(y_raw, r_f, st, MEAS)
    if isnan(st.y_filt)
        st.y_filt = y_raw;
    end
    st.y_filt = (1 - MEAS.alpha_y) * st.y_filt + MEAS.alpha_y * y_raw;
    y_corr = st.y_filt - st.bhat;
    e = r_f - y_corr;
    st.bhat = (1 - MEAS.bias_leak) * st.bhat - MEAS.gamma_b * e;
    y_used = st.y_filt - st.bhat;
end
