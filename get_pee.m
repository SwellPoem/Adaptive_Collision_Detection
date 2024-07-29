function pee = get_pee(q1,q2,q3)

t2 = cos(q2);
t3 = q2+q3;
t4 = t2.*5.0;
t5 = cos(t3);
t6 = t5.*4.0;
t7 = t4+t6;
pee = [(t7.*cos(q1))./1.0e+1;(t7.*sin(q1))./1.0e+1;sin(q2)./2.0+sin(t3).*(2.0./5.0)+1.0./2.0];
