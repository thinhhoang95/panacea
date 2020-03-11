file = fopen('pos.txt','w');
for i=1:length(out.pos.time)
    fprintf(file, '%f,%f,%f,%f\r\n', out.pos.time(i), out.pos.signals.values(i,:));
end
fclose(file);

fprintf('Position have been written to file successfully!');