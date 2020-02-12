if exist('ser','var')
    fclose(ser);
    delete(ser);
    clear ser;
end
ser=serial('COM11');
fopen(ser);

while 1
    str = fgetl(ser);
    fprintf(1,'%s\n',str);
    q=str2num(str);
    
    q = quatnormalize(q);
    
end
