function bs2_read(port,fcncallback)

%if exist('ser','var')
%     fclose(ser);
%     delete(ser);
%     clear ser;
%end

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

ser=serial(port);
fopen(ser);

while 1
    str = fgetl(ser);
    %fprintf(1,'%s\n',str);
    q=str2num(str);
    
    if length(q)~=4
        continue;
    end
   
   
    q = quatnormalize(q);

    
    fcncallback(q);
    
end
