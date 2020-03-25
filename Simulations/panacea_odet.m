function [Xout] = panacea_odet(Add, Bdd, imuTs, detectionTs, out, window_length, f, a_process, Xout_accel)
    close all;
    fprintf('Starting Panacea Algorithm for Object Detection... \n');
    fprintf('State Estimation for Autonomous Helicopter Landing by Hoang Dinh Thinh (1970062) \n');
    fprintf('Author: Hoang Dinh Thinh \n');
    fprintf('Starting memory alloc... ');
    % global Aa Ba imuTs trackingTs out N_window Q R;
    % Allocation of variables
    X = zeros(6,length(out.accel.time));
    P = zeros(6,6,length(out.accel.time));
    Xww = zeros(6,window_length);
    Pww = zeros(6,6,window_length);
    Rebww = zeros(3,3,window_length);
    uw = zeros(3,window_length);
    rez = zeros(3,length(out.xiod.time));
    fitness = 0;
    fprintf('OK! \n');
    fprintf('Initializing window variables... ');
    % R: covariance of the acceleration, Q: measurement of detection
    R = 0.5*eye(3);
    Q = diag([1e-4 1e-4 7]);
    Qi = inv(Q);
    imu_cursor = 1; % cursor of the current state in the Xw
    detection_cursor = 1;
    next_detection_clock = out.xiod.time(detection_cursor + 1);
    will_adjust_cursor = false;
    fprintf('OK! \n Panacea ODE is started... ');
    for time=1:length(out.accel.time)
        % For each IMU data received, perform reckoning of the new position
        % and velocity:
        % Firstly, transform the acceleration in body frame to inertial
        % frame
        Reb = eul2rotm([out.eulang.Data(time,3) out.eulang.Data(time,2) out.eulang.Data(time,1)]);
        clock = time * imuTs;
        a = out.accel.signals.values(time,:);
        % a = a + a_process(time,:);
        a = a + mvnrnd([0 0 0], 0.5*eye(3));
        imu_cursor = imu_cursor + 1;
        Xww(:,imu_cursor) = Add * Xww(:,imu_cursor - 1) + Bdd * a';
        Pww(:,:,imu_cursor) = Add * Pww(:,:,imu_cursor - 1) * Add' + Bdd * R * Bdd';
        Rebww(:,:,imu_cursor) = Reb;
        uw(:,imu_cursor - 1) = a';
        if (clock>next_detection_clock)
            % Once AI detection result is available, perform the correction
            % step 
            % detection_cursor indicates the IMU state which object
            % detection starts, and detection_cursor+1 is where it ends
            % We refine the first state of the window 
            xes = out.xiod1.Data(:,:,detection_cursor);
            xe = xes(1);
            ye = xes(2);
            F1 = [f, 0, -xe; 0, f, -ye; 0, 0, -1];
            F2 = [eye(3), zeros(3)];
            H = F1*Rebww(:,:,1)*F2;
            disp(Rebww(:,:,1));
            state = Xww(:,1); % the first state is refined
            state_true = out.pos.signals.values(time-imu_cursor+2,:);
            % fprintf('Difference between state and true state: \n');
            fitness = fitness + norm((state(1:3))' - state_true);
            P = Pww(:,:,1);
            z = [0; 0; -out.xtod.Data(detection_cursor,3)];
            if (det(P)<1e-12)
                disp(det(P));
                % The matrix is near singular, skip the correction step
                Xww(:,1) = state;
                Pww(:,:,1) = P;
            else
                % S = H*P*H'+Q;
                % Kg = P*H'*inv(S);
                % state_hat = state + Kg * (z - H*state);
                rez(:,detection_cursor) = z-F1*state(1:3);
                % fprintf('Normalized error: %.2f \n', norm(z-H*state));
                % covar_hat = (eye(size(state,1)) - Kg*H)*P;
                %%% state_hat = state;
                %%% covar_hat = P;
                Pi = inv(P);
                state_hat = inv(H'*Qi*H + Pi) * (H'*Qi*z + Pi*state); % maximum-a-posteriori estimation
                % Make state_hat the first state of the window
                Xww(:,1) = state_hat;
                % fprintf('State at %d second is: \n', next_detection_clock);
                % disp(state_hat);
                % fprintf('Cursor is: %d \n', imu_cursor);
                covar_hat = inv(H'*Qi*H + Pi);
                Pww(:,:,1) = covar_hat;
                % Propagate the new estimation using acceleration upto the
                % present
                for q=1:imu_cursor - 1
                    Xww(:,q+1) = Add * Xww(:,q) + Bdd * uw(:,q);
                    Pww(:,:,q+1) = Add * Pww(:,:,q) * Add' + Bdd * R * Bdd';
                end
            end
            will_adjust_cursor = true;
            detection_cursor = detection_cursor + 1;
            if (detection_cursor >= length(out.xiod.time))
                detection_cursor = detection_cursor - 1;
            else
                next_detection_clock = out.xiod.time(detection_cursor + 1);
            end
        end
        X(:,time) = Xww(:,imu_cursor);
        P(:,:,time) = Pww(:,:,imu_cursor);
        if (will_adjust_cursor == true)
            % Set the first state of the window to the START OF THE
            % DETECTION ALGORITHM, in this case, the latest IMU state
            Xww(:,1) = Xww(:,imu_cursor);
            Pww(:,:,1) = Pww(:,:,imu_cursor);
            Rebww(:,:,1) = Rebww(:,:,imu_cursor);
            imu_cursor = 1;
            will_adjust_cursor = false;
        end
    end
    Xout = X;
    fprintf('Done! Inverse-Fitness is %.2f \n', fitness);
    fprintf('Done! \n Plotting data... ');
    figure
    plot(out.accel.time, Xout(1:3,:)');
    hold on
    plot(out.pos.time,out.pos.signals.values,'--');
    plot(out.accel.time,Xout_accel(1:3,:),'-.');
    hold off
    figure
    plot(rez');
    fprintf('Done! \n');
end