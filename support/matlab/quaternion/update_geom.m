function update_geom(h,vcoord,vconnect,vcol)

for i=1:size(vconnect,1)
    c1 = vcoord(vconnect(i,1),:);
    c2 = vcoord(vconnect(i,2),:);
    
    l = h(i);
    set(l,'XData',[c1(1) c2(1)]);
    set(l,'YData',[c1(2) c2(2)]);
    set(l,'ZData',[c1(3) c2(3)]);
    
end

