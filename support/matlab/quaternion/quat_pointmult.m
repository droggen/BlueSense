%Quaternion multiplication without the .a component. Returns a vector
function result=quat_pointmult(q1,q2)
    result=[0 0 0];

    result(1) = (q1(1)*q2(2) +q1(2)*q2(1) +q1(3)*q2(4) -q1(4)*q2(3));
    result(2) = (q1(1)*q2(3) -q1(2)*q2(4) +q1(3)*q2(1) +q1(4)*q2(2));
    result(3) = (q1(1)*q2(4) +q1(2)*q2(3) -q1(3)*q2(2) +q1(4)*q2(1));
end