function [Y, X] = regressor_matrix(M, C, G, tau_f, fv, m, I, l, dc, g0, q1_ddot, q2_ddot, q3_ddot)
   
    syms a1 a2 a3 a4 a5 a6 a7 a8 a9 a10 a11 real

    a = [a1;a2;a3;a4;a5;a6;a7;a8;a9;a10;a11];

    q_ddot =[q1_ddot; q2_ddot; q3_ddot];

    u=M*q_ddot+C+G+tau_f;

    Y=jacobian(u,a);

    x1 = I(3,3,1)+I(1,1,2)+I(1,1,3);
    x2 = I(2,2,2)+m(2)*dc(2)^2+m(3)*l(2)^2-I(1,1,2);
    x3 = I(2,2,3)+m(3)*dc(3)^2-I(1,1,3);
    x4 = m(3)*l(2)*dc(3);
    x5 = m(3)*dc(3)^2+I(2,2,2)+m(2)*dc(2)^2+I(2,2,3)+m(3)*l(2)^2;  
    x6 = I(2,2,3)+m(3)*dc(3)^2;
    x7 = m(2)*dc(2)*g0+m(3)*l(2)*g0;
    x8 = m(3)*dc(3)*g0;
    x9 = fv(1);
    x10 = fv(2);
    x11 = fv(3);
    
    X = [x1; x2; x3; x4; x5; x6; x7; x8; x9; x10; x11];

end