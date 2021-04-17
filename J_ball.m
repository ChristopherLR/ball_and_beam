function J = J_ball(x)
    m_ball = 80.44e-3;
    r_ball = 13.5e-3;
    J_ball_static = (2/5)*m_ball*(r_ball^2);
    J = J_ball_static + m_ball*x^2;
end