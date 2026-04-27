function rapid_plot_measurement_bias_diagnostics(OUT)
    mask = ~isnan(OUT.t_meas);
    t_hr = OUT.t_meas(mask) / 60;
    y_true = interp1(OUT.time, OUT.T4_log, OUT.t_meas(mask), 'linear');
    y_raw  = OUT.T4_meas_raw(mask);
    y_used = OUT.T4_meas_used(mask);
    bhat   = OUT.bias_hat_hist(mask);

    figure('Color','w','Position',[120 120 1100 700]);

    subplot(2,1,1); hold on;
    plot(t_hr, y_true, '-r', 'LineWidth', 1.8, 'DisplayName', 'y_{true}');
    plot(t_hr, y_raw,  '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.2, 'DisplayName', 'y_{raw}');
    plot(t_hr, y_used, '-', 'Color', [0.85 0.5 0], 'LineWidth', 1.8, 'DisplayName', 'y_{used}');
    yline(OUT.T4_des, '--k', 'LineWidth', 1.2, 'DisplayName', 'Setpoint');
    xlabel('Time (h)'); ylabel('T_4'); title('Measurement diagnostics');
    legend('Location','best'); grid on;

    subplot(2,1,2); hold on;
    plot(t_hr, bhat, '-b', 'LineWidth', 1.8, 'DisplayName', '\hat b');
    plot(t_hr, y_used - y_true, '--m', 'LineWidth', 1.4, 'DisplayName', 'y_{used} - y_{true}');
    yline(0, ':k', 'LineWidth', 1.0);
    xlabel('Time (h)'); ylabel('Bias / gap');
    title('Bias estimate and corrected-measurement error');
    legend('Location','best'); grid on;
end
