function panacea_hochiminhzhi(Aa, Add, Ba, Bdd, imuTs, trackingTs, out, N_window, Q, R)
    % global Aa Ba imuTs trackingTs out N_window Q R;
    % Initialize the variables and initial conditions
    Xa = zeros((N_window+2)*6,1,length(out.accel.time));
    Xa_minus = zeros((N_window+2)*6,1,length(out.accel.time));
    Xa_plus = zeros((N_window+2)*6,1,length(out.accel.time));
    Pa = zeros(6,6,length(out.accel.time));
    Pa_minus = zeros(6,6,length(out.accel.time));
    Pa_plus = zeros(6,6,length(out.accel.time));
    % Propagate the state forward by IMU measurement
    time_max = length(out.accel.time) - 1;
    for time=1:time_max % For each IMU timestep
        u = out.accel.signals.values(time,:);
        Xa_minus(:,:,time+1) = Aa * Xa(:,time) + Ba * u';
        Pa_minus(:,:,time+1) = Add * Pa(:,:,time) * Add' + Bdd * Q * Bdd';
        Xa = Xa_minus;
        Pa = Pa_minus;
    end
    % Final true state: 8.5882 -8.5882 -30.7279
    fprintf('True final state: \n');
    disp(Xa(1:3,time_max+1));
    
end