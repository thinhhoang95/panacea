file = fopen('eulang.txt','w');
Rc = eul2rotm(pi/180*[6 -8.5 20]);
for i=1:length(out.eulang.Time)
    prc = mvnrnd([0 0 0], 0.05*eye(3));
    R = eul2rotm(out.eulang.Data(i,:));
    R = Rc*R;
    rpy = rotm2eul(R);
    fprintf(file, '%f,%f,%f,%f\r\n', out.eulang.Time(i), rpy);
end
fclose(file);

fprintf('Euler Angles have been written to file successfully! \n');