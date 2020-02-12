function vr = rotate_geom(v,quat)

vr=v;

for i=1:size(v,1)
    vr(i,:) = quatrotate(quat,vr(i,:));
    %vr(i,:) = rot_quat(vr(i,:),quat);
end

