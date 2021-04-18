function T = T_ball(x, theta)
    m_ball = 80.44e-3;
    g = 9.81;
    T = x*g*m_ball*cos(theta);
end