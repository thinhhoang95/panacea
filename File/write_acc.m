file = fopen('accel.txt','w');
Rc = eul2rotm(pi/180*[6 -8.5 20]);
clock_eulang = 0;
cursor_eulang = 1;
for i=1:length(out.accel.time)
    clock = out.accel.time(i);
    while (clock_eulang < clock && cursor_eulang<=length(out.eulang.Time))
        cursor_eulang = cursor_eulang + 1;
        clock_eulang = out.eulang.Time(cursor_eulang);
    end
    eulang = out.eulang.Data(cursor_eulang, :);
    R = eul2rotm(eulang);
    n = mvnrnd([0 0 0],0.0005*eye(3));
    % fprintf(file, '%f,%f,%f,%f\r\n', out.accel.time(i), (Rc*out.accel.signals.values(i,:)' + Rc*R*n')');
    fprintf(file, '%f,%f,%f,%f\r\n', out.accel.time(i), (Rc*out.accel.signals.values(i,:)')');
end
fclose(file);

fprintf('Acceleration have been written to file successfully! \n');