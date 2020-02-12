% Rotates vector v by quaternion q=[q0,q1,q2,q3]=[cos(a/2);sin(a/2)x,sin(a/2)y,sin(a/2)z]
% result = q*v*q' with q' the conjugate
function result = quat_rot(q,v)
    qconj = [0 0 0 0];                  % conjugate of the rotation quaternion
    qv = [0 0 0 0];                     % quaternion representation of the vector to rotate

    qv(1) = 0;
    qv(2) = v(1);
    qv(3) = v(2);
    qv(4) = v(3);
    qconj(1) = q(1);
    qconj(2) = -q(2);
    qconj(3) = -q(3);
    qconj(4) = -q(4);
    
    result =quat_pointmult(quat_mult(q,qv),qconj);
end