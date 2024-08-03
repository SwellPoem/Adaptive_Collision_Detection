function [M, C, G, M_par, C_par, G_par] = dynamics(m, I, l, dc, g0, q1, q2, q3)

    syms q1_dot q2_dot q3_dot real;
    syms a1 a2 a3 a4 a5 a6 a7 a8 real
    
    q_sym = [q1; q2; q3];
    q_dot_sym = [q1_dot; q2_dot; q3_dot];

    T1=(1/2)*I(3,3,1)*q1_dot^2;

    pc2=[dc(2)*cos(q2)*cos(q1) dc(2)*cos(q2)*sin(q1) l(1)+dc(2)*sin(q2)]';
    vc2=simplify(diff(pc2,q1)*q1_dot+diff(pc2,q2)*q2_dot);
    T2c=(1/2)*simplify(m(2)*vc2'*vc2);

    om2=[q1_dot*sin(q2) q1_dot*cos(q2) q2_dot]';
    T2a=(1/2)*om2'*diag([I(1,1,2) I(2,2,2) I(2,2,2)])*om2;

    T2=simplify(T2c+T2a);

    pc3=[(l(2)*cos(q2)+dc(3)*cos(q2+q3))*cos(q1) (l(2)*cos(q2)+dc(3)*cos(q2+q3))*sin(q1) l(1)+l(2)*sin(q2)+dc(3)*sin(q2+q3)]';
    vc3=simplify(diff(pc3,q1)*q1_dot+diff(pc3,q2)*q2_dot+diff(pc3,q3)*q3_dot);
    T3c=(1/2)*simplify(m(3)*vc3'*vc3);

    om3=[q1_dot*sin(q2+q3) q1_dot*cos(q2+q3) q2_dot+q3_dot]';
    T3a=(1/2)*om3'*diag([I(1,1,3) I(2,2,3) I(2,2,3)])*om3;

    T3=simplify(T3c+T3a);

    T=simplify(T1+T2+T3);

    M(1,1)=diff(T,q1_dot,2);
    TempM1=diff(T,q1_dot);
    M(1,2)=diff(TempM1,q2_dot);
    M(1,3)=diff(TempM1,q3_dot);
    M(2,2)=diff(T,q2_dot,2);
    TempM2=diff(T,q2_dot);
    M(2,3)=diff(TempM2,q3_dot);
    M(3,3)=diff(T,q3_dot,2);
    M(2,1)=M(1,2);
    M(3,1)=M(1,3);
    M(3,2)=M(2,3);
    M=simplify(M);
   
    %parametrization for the regressor matrix

    %a1=I1zz+I2xx+I3xx 
    %a2=I2yy+m2*dc2^2+m3*L2^2-I2xx
    %a3=I3yy+m3*dc3^2-I3xx
    %a4=m3*L2*dc3
    %a5=I2yy+m2*dc2^2+m3*dc3^2+I3yy+m3*L2^2
    %a6=I3yy+m3*dc3^2
    % 
    M_par(3,3)=subs(M(3,3),I(2,2,3)+m(3)*dc(3)^2,a6);
    M_par(2,3)=subs(M(2,3),{I(2,2,3)+m(3)*dc(3)^2,m(3)*l(2)*dc(3)},{a6,a4});
    M_par(3,2)= M_par(2,3);
    M_par(2,2)=subs(M(2,2),{I(2,2,3)+I(2,2,2)+m(3)*dc(3)^2+m(2)*dc(2)^2+m(3)*l(2)^2,m(3)*l(2)*dc(3)},{a5,a4});
    %special treatment for element (1,1)
    M_par11=M(1,1);
    M_par11=subs(M_par11,{cos(2*q2 + 2*q3),cos(2*q2)},{2*(cos(q2+q3))^2-1,2*(cos(q2))^2-1});
    M_par11=subs(M_par11,{sin(q2)^2},{1-cos(q2)^2});
    M_par11=simplify(M_par11);
    M_par11=collect(M_par11,'cos');
    M_par11=subs(M_par11,{I(3,3,1)+I(1,1,2)+I(1,1,3),m(3)*l(2)*dc(3)},{a1,a4});
    M_par11=subs(M_par11,{I(2,2,2)+m(2)*dc(2)^2+m(3)*l(2)^2-I(1,1,2),I(2,2,3)+m(3)*dc(3)^2-I(1,1,3)},{a2,a3});
    M_par(1,1)=M_par11;
    M_par = simplify(M_par);


    M1=M(:,1);
    C1=(1/2)*(jacobian(M1,q_sym)+jacobian(M1,q_sym)'-diff(M,q1));
    M2=M(:,2);
    C2=(1/2)*(jacobian(M2,q_sym)+jacobian(M2,q_sym)'-diff(M,q2));
    M3=M(:,3);
    C3=(1/2)*(jacobian(M3,q_sym)+jacobian(M3,q_sym)'-diff(M,q3));

    c1=simplify(q_dot_sym'*C1*q_dot_sym);
    c2=simplify(q_dot_sym'*C2*q_dot_sym);
    c3=simplify(q_dot_sym'*C3*q_dot_sym);

    C = simplify([c1; c2; c3]);

    M1_par=M_par(:,1);
    C1_par=(1/2)*(jacobian(M1_par,q_sym)+jacobian(M1_par,q_sym)'-diff(M_par,q1));
    M2_par=M_par(:,2);
    C2_par=(1/2)*(jacobian(M2_par,q_sym)+jacobian(M2_par,q_sym)'-diff(M_par,q2));
    M3_par=M_par(:,3);
    C3_par=(1/2)*(jacobian(M3_par,q_sym)+jacobian(M3_par,q_sym)'-diff(M_par,q3));

    c1_par=simplify(q_dot_sym'*C1_par*q_dot_sym);
    c2_par=simplify(q_dot_sym'*C2_par*q_dot_sym);
    c3_par=simplify(q_dot_sym'*C3_par*q_dot_sym);

    C_par = simplify([c1_par; c2_par; c3_par]);

    g=[0;0;-g0];

    U1=0;
    U2=-m(2)*g'*pc2;
    U3=-m(3)*g'*pc3;
    U=simplify(U1+U2+U3);

    G=simplify(jacobian(U,q_sym))';

    %a7=(m2*dc2+m3*L2)*g0
    %a8=m3*dc3*g0

    G_par=collect(G,'cos');
    G_par(3)=subs(G_par(3),{m(3)*dc(3)*g0},{a8});
    G_par(2)=subs(G_par(2),{m(2)*dc(2)*g0+m(3)*l(2)*g0,m(3)*dc(3)*g0},{a7,a8});

    G_par = simplify(G_par);

end

