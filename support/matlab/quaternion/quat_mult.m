function result=quat_mult(q1,q2)
    result=[0 0 0 0];
    result(1) = (q1(1)*q2(1) -q1(2)*q2(2) -q1(3)*q2(3) -q1(4)*q2(4));
    result(2) = (q1(1)*q2(2) +q1(2)*q2(1) +q1(3)*q2(4) -q1(4)*q2(3));
    result(3) = (q1(1)*q2(3) -q1(2)*q2(4) +q1(3)*q2(1) +q1(4)*q2(2));
    result(4) = (q1(1)*q2(4) +q1(2)*q2(3) -q1(3)*q2(2) +q1(4)*q2(1));
end