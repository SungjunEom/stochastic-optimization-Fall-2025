% "move ... to the nearest valid point within Θ"
function theta_clamped = clamp_to_limits(theta, q_min, q_max)
    theta_clamped = max(q_min, min(q_max, theta));
end