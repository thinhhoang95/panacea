file = fopen('eulang_true.txt','w');
for i=1:length(out.eulang.Time)
    % prc = mvnrnd([0 0 0], 0.05*eye(3));
    % R = eul2rotm(out.eulang.Data(i,:));
    % R = R*Rc';
    % rpy = rotm2eul(R);
    fprintf(file, '%f,%f,%f,%f\r\n', out.eulang.Time(i), out.eulang.Data(i,:));
end
fclose(file);

fprintf('TRUE Euler Angles have been written to file successfully! \n');