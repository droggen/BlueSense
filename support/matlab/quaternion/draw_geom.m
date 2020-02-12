function h = draw_geom(vcoord,vconnect,vcol)

h=[];

for i=1:size(vconnect,1)
    c1 = vcoord(vconnect(i,1),:);
    c2 = vcoord(vconnect(i,2),:);
    
    t=line([c1(1) c2(1)],[c1(2) c2(2)],[c1(3) c2(3)],'Color',vcol(i));
    set(t,'LineWidth',4);
    h=[h t];
end

