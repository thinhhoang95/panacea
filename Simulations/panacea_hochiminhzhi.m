function panacea_hochiminhzhi(Aa, Add, Ba, Bdd, imuTs, trackingTs, out, N_window, Q, R)
    % global Aa Ba imuTs trackingTs out N_window Q R;
    Xa = zeros(length(out.accel.time),1,(N_window+2)*6);
    Xa_minus = zeros(length(out.accel.time),1,(N_window+2)*6);
    Xa_plus = zeros(length(out.accel.time),1,(N_window+2)*6);
    Pa = zeros(length(out.accel.time),6,6);
    Pa_minus = zeros(length(out.accel.time),6,6);
    Pa_plus = zeros(length(out.accel.time),6,6);
    % Propagate the state forward by IMU measurement
    for time=1:length(out.accel.time)-1 % For each IMU timestep
        u = out.accel.signals.values(time,:);
        Xa_minus(time+1,:,:) = Aa * Xa(time,:)' + Ba * u';
        Pa_minus(time+1,:,:) = Add * Pa(time,:,:) * Add' + Bdd * Q * Bdd';
    end
end