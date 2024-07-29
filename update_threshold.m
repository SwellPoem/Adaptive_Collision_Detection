function threshold = update_threshold(r)
    % Update threshold based on residuals
    threshold = max(abs(r)) * 1.1;
end