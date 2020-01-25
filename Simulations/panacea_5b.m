function [dbg_meas,dbg_ofd,dbg_csr,dbg_acc,X_out,X_true,P_out,accuracy] = panacea_5b(Aa, Add, Ba, Bdd, imuTs, trackingTs, out, N_window, Q, R_val, f, accel_rndprc)
    close all;
    fprintf('Starting Panacea Algorithm... \n');
    fprintf('State Estimation for Autonomous Helicopter Landing by Hoang Dinh Thinh (1970062) \n');
    fprintf('Author: Hoang Dinh Thinh \n');
    fprintf('Starting memory alloc... ');
    % global Aa Ba imuTs trackingTs out N_window Q R;
    % Initialize debug variables
    dbg_meas = zeros(1,length(out.ofd.time));
    dbg_ofd = zeros(1,length(out.ofd.time));
    dbg_csr = zeros(1,length(out.ofd.time));
    dbg_acc = zeros(1,length(out.ofd.time));
    accuracy = zeros(length(out.ofd.time),1);
    accuracy_raw = zeros(length(out.accel.time),1);
    avg_ofd = zeros(length(out.ofd.time),1);
    avg_disturb = zeros(length(out.ofd.time),1);
    % Initialize the variables and initial conditions
    Xa = zeros((N_window+2)*6,1,length(out.accel.time));
    Xa_true = zeros((N_window+2)*6,1,length(out.accel.time));
    Xa_minus = zeros((N_window+2)*6,1,length(out.accel.time));
    Xa_raw = zeros((N_window+2)*6,1,length(out.accel.time));
    Xa_plus = zeros((N_window+2)*6,1,length(out.accel.time));
    Pa = zeros((N_window+2)*6,(N_window+2)*6,length(out.accel.time));
    Pa_minus = zeros((N_window+2)*6,(N_window+2)*6,length(out.accel.time));
    Pa_plus = zeros((N_window+2)*6,(N_window+2)*6,length(out.accel.time));
    time_max = length(out.accel.time) - 1;
    % time_max = 200;
    next_tracking_time = 2;
    next_tracking_clock = out.psi.Time(next_tracking_time); % Only apply Panacea after 3 seconds
    tracking_delay_length = trackingTs/imuTs;
    fprintf('OK \n');
    % Corrupt the acceleration signals
    accel_corrupted = out.accel.signals.values;
    for i=1:length(out.accel.time)
        accel_corrupted(i,:) = accel_corrupted(i,:) + 10*accel_rndprc(i,:);
    end
    % Discretize the optical flow measurement
    du = 0.059375e-3; % smallest possible optical flow measurement
    for i=1:length(out.ofd.time)
        out.ofd.signals.values(:,:,i) = floor(out.ofd.signals.values(:,:,i)./du) * du;
    end
    % Corrupt the optical flow measurement with a uniform distribution of
    % 10px
    fprintf('Processing measurement data... ');
    for i=1:length(out.ofd.time)
        z = out.xi.Data(i,3);
        pixelVar = abs(floor(43.79/z));
        avg_disturb(i) = pixelVar;
        for lm=1:length(out.ofd.signals.values(:,:,i))
            out.ofd.signals.values(lm,:,i) = out.ofd.signals.values(lm,:,i) + (-pixelVar*0.059375e-3+2*round(rand([1 2])*pixelVar)*0.059375e-3);
        end
    end
    clear i lm pixelVar z;
    fprintf('OK! \n');
    % >> Beginning of the algorithm <<
    for time=1:time_max % For each IMU timestep, time is the multiplier of imuTs
        clock = time*imuTs;
        % Propagate the state forward by IMU measurement
        u = accel_corrupted(time,:);
        u_true = out.accel.signals.values(time,:);
        if (clock>3)
            % u = u + mvnrnd([0 0 0],Q);
        end
        Xa_raw(:,time+1) = Aa * Xa_raw(:,time) + Ba * u';
        Xa_minus(:,time) = Aa * Xa(:,time) + Ba * u';
        Xa_true(:,time+1) = Aa * Xa_true(:,time) + Ba * u_true';
        Pa_minus(:,:,time) = Aa * Pa(:,:,time) * Aa' + Ba * Q * Ba';
        % Search for the entry in the trackingTs samples
        if (clock>next_tracking_clock && clock>3)
            fprintf('Measurement vector and matrix %f... ', clock);
            % The anti-yawing matrices
            psi_f = -out.psi.Data(next_tracking_time);
            psi_i = -out.psi.Data(next_tracking_time-1);
            Y_f = [cos(psi_f) -sin(psi_f) 0; sin(psi_f) cos(psi_f) 0; 0 0 1];
            Y_i = [cos(psi_i) -sin(psi_i) 0; sin(psi_i) cos(psi_i) 0; 0 0 1];
            % Preprocess the optical flow vector
            ofdx = [];
            xex = [];
            ofdv = out.ofd.signals.values(:,:,next_tracking_time);
            xev = out.xe.signals.values(:,:,next_tracking_time);
            for i=1:length(ofdv)
                if (ofdv(i,1)~=0 || ofdv(i,2)~=0)
                    ofdx = [ofdx; ofdv(i,:)];
                    xex = [xex; xev(i,:)];
                end
            end
            of_pix = ofdx./du;
            avg_ofd(next_tracking_time)=norm(mean(of_pix,1));
            clear ofdv xev i;
            % Calculate the measurement vector z of the optical flow
            % Measurement vector includes optical flow and the acceleration
            % between subsequent past IMU measurements and direct
            % observation of origin optical flow state e.g. X(-5)
            z = zeros(length(ofdx)*2+3*tracking_delay_length+6,1);
            H = zeros(size(z,1),size(Xa,1));
            Sf1 = [eye(3) zeros(3)]; % selecting position states out of 6 states of position and velocity
            Sf2 = zeros(6,6*(N_window+2));
            % >>>>> IF THE FIRST STATE IS NOT THE STATE THAT USES TO
            % CALCULATE THE OPTICAL FLOW, MODIFY THE NEXT LINE <<<<<
            final_of_state = 0;
            Sf2(final_of_state*6+1:(final_of_state+1)*6,final_of_state*6+1:(final_of_state+1)*6) = eye(6); % THE FIRST STATE IS THE FINAL STATE OF THE OPTICAL FLOW
            x_f = Xa_minus(final_of_state*6+1:(final_of_state+1)*6,time); % THE FIRST STATE IS THE FINAL STATE OF THE OPTICAL FLOW
            % >>>>> <<<<< %
            x_i = Sf1*Xa(tracking_delay_length*6+1:(tracking_delay_length+1)*6,time);
            % x_i = out.xi.Data(next_tracking_time,:);
            % x_f = out.xf.Data(next_tracking_time,:);
            for i=1:length(ofdx)
                % Fill in the optical flow measurement components
                dx = ofdx(i,1);
                dy = ofdx(i,2);
                xe = xex(i,1);
                ye = xex(i,2);
                F = [f 0 -(xe+dx); 0 f -(ye+dy)];
                z((i-1)*2+1:i*2) = 1e3 * F*Y_f*((-x_i(3)/f) * inv(Y_i) * [xe ye 0]' + [x_i(1) x_i(2) 0]');
                H((i-1)*2+1:i*2,:) = 1e3 * F*Y_f*Sf1*Sf2;
            end
            clear dx dy xe ye F i;
            
            for i=1:tracking_delay_length
                % Fill in the acceleration measurement
                z(length(ofdx)*2+(i-1)*3+1:length(ofdx)*2+i*3) = accel_corrupted(time-(i-1),:) * imuTs;
                % Fill in acceleration observation matrix
                for j=1:3 % vx, vy, vz
                    H(length(ofdx)*2+(i-1)*3+j,(i-1)*6+4+(j-1):i*6+4+(j-1))=[1 0 0 0 0 0 -1];
                end
                % Fill in constrained origin state of the optical flow
                % analysis (i.e. X(-5))
                z(length(ofdx)*2+3*tracking_delay_length+1:length(ofdx)*2+3*tracking_delay_length+6) = Xa_minus(size(Xa_minus,1)-5:size(Xa_minus,1),time);
                H(length(ofdx)*2+3*tracking_delay_length+1:length(ofdx)*2+3*tracking_delay_length+6,6*tracking_delay_length+1:6*(tracking_delay_length+1)) = eye(6);
            end
            clear i j;
            
            Ra = zeros(size(z,1));
            % Covariance of measurement uncertainty
            
            Ra(1:length(ofdx)*2,1:length(ofdx)*2) = 10 * R_val * eye(length(ofdx)*2);
            % Covariance of the acceleration constraints between states
            for i=1:tracking_delay_length
                Ra(length(ofdx)*2+(i-1)*3+1:length(ofdx)*2+i*3,length(ofdx)*2+(i-1)*3+1:length(ofdx)*2+i*3) = Q;
            end
            % The covariance related to the initial constraint of the
            % original state of the optical flow analysis (X(-5)) has zero
            % measurement uncertainty (since it is the prior to base the
            % observation of the optical flow). Ra is initialized from
            % zero, thus left intact!
            
            Ra(length(ofdx)*2+tracking_delay_length*3+1:length(ofdx)*2+tracking_delay_length*3+6,length(ofdx)*2+tracking_delay_length*3+1:length(ofdx)*2+tracking_delay_length*3+6) = Pa_minus(tracking_delay_length*6+1:(tracking_delay_length+1)*6,tracking_delay_length*6+1:(tracking_delay_length+1)*6,time);
            
            clear i;
            
            fprintf('OK \n');
            
            fprintf('Kalman filtering... ');
            K_gain = Pa_minus(:,:,time) * H' / (H * Pa_minus(:,:,time) * H' + Ra);
            innov = z - H*Xa_minus(:,time);
            adjust = K_gain * innov;
            % adjust = zeros(size(Xa,1),1);
            Xa_plus(:,time) = Xa_minus(:,time) + adjust;
            Pa_plus(:,:,time) = Pa_minus(:,:,time) * (eye((N_window+2)*6) - H'*K_gain');
            fprintf('OK \n');
            dbg_meas(next_tracking_time) = norm(innov);
            dbg_ofd(next_tracking_time) = norm(innov(1:length(ofdx)*2));
            dbg_csr(next_tracking_time) = norm(innov(length(ofdx)*2+tracking_delay_length*3+1:length(ofdx)*2+tracking_delay_length*3+6));
            dbg_acc(next_tracking_time) = norm(innov(length(ofdx)*2+1:length(ofdx)*2+tracking_delay_length*3));
            fprintf('Measurement error: %.16f \n', dbg_meas(next_tracking_time));
            fprintf('Panacea adjusted at time %f / %d: \n', [clock, next_tracking_time]);
            disp(adjust(tracking_delay_length*6+1:(tracking_delay_length+1)*6-3));
            disp(adjust(1:3));
            % Moves the next trackingTs processing moment forward
            next_tracking_time = next_tracking_time + 1;
            next_tracking_clock = out.psi.Time(next_tracking_time);
        elseif (clock>next_tracking_clock && clock<=3)
            next_tracking_time = next_tracking_time + 1;
            next_tracking_clock = out.psi.Time(next_tracking_time);
            Xa_plus(:,time) = Xa_minus(:,time);
            Pa_plus(:,:,time) = Pa_minus(:,:,time);
        else
            Xa_plus(:,time) = Xa_minus(:,time);
            Pa_plus(:,:,time) = Pa_minus(:,:,time);
        end
        Xa(:,time+1) = Xa_plus(:,time);
        Pa(:,:,time+1) = Pa_plus(:,:,time);
        % Evaluate the accuracy of the solution
        accuracy(time+1,:) = norm(Xa(1:3,time+1) - Xa_true(1:3,time+1));
        accuracy_raw(time+1,:) = norm(Xa_raw(1:3,time+1) - Xa_true(1:3,time+1));
        % fprintf('Corrected state at time %f: \n', time);
        % disp(Xa(:,time+1));
    end
    fprintf('Final true state: 8.5882 -8.5882 -30.7279 \n');
    fprintf('True final state: \n');
    disp(Xa(1:3,time_max));
    X_out = Xa(1:6,:,:);
    X_true = Xa_true(1:6,:,:);
    P_out = Pa(tracking_delay_length*6+1:(tracking_delay_length+1)*6,tracking_delay_length*6+1:(tracking_delay_length+1)*6,:);
    fprintf('Plotting data... \n');
    figure
    subplot(3,2,1)
    plot(X_out(1:3,:)');
    title('Estimation of states');
    xlabel('Time samples');
    ylabel('Distance (m)');
    subplot(3,2,2)
    plot(X_true(1:3,:)');
    title('Ground-truth states');
    xlabel('Time samples');
    ylabel('Distance (m)');
    subplot(3,2,3)
    plot(out.accel.time,accuracy);
    title('Error');
    xlabel('Time (s)');
    ylabel('Distance (m)');
    subplot(3,2,4)
    plot(out.accel.time,Xa_raw(1:3,:)');
    title('Acceleration integration');
    xlabel('Time (s)');
    ylabel('Distance (m)');
    subplot(3,2,5)
    plot(out.accel.time, accuracy_raw');
    title('Dead-reckoning accuracy');
    xlabel('Time (s)');
    ylabel('Distance (m)');
    subplot(3,2,6)
    plot(permute(P_out(1,1,:),[3 1 2]));
    hold on
    plot(permute(P_out(2,2,:),[3 1 2]));
    plot(permute(P_out(3,3,:),[3 1 2]));
    title('Covariance elements');
    xlabel('Time samples');
    ylabel('Covariance (m^2)');
    hold off
    figure
    plot(out.ofd.time,avg_ofd);
    title('Optical flow average distance');
    xlabel('Time (s)');
    ylabel('Optical flow (px)');
    figure
    plot(out.ofd.time,avg_disturb);
    title('Average blurring process');
    xlabel('Time (s)');
    ylabel('Blur (px)');
end