% test_driver.m
% Random Walk driver and EKF demonstration

clear all;
close all;

r_pos=[0,0;
    0,10;
    10,10;
    10,0];
h=trilateration_transform(r_pos);
disp(h);

% provide initial position, covariance, sensor covariance, label, and
% beacon positions
q_init=[5;5];
% Initialize p super high. Will make a bad prediction initially, but
% convergence is virtually guaranteed
p_init=diag([1000000 1000000 1000000 1000000]);

% Process and measurement covariances
% Higher process covariance -> trust kinematic model (dead reckoning) less
covs=eye(4)*.001;
% Higher noise covariance -> trust measurements less
R=eye(4)*1;

label="uwb1";
uwbpos=r_pos;
uwb1_ekf=ekf(q_init,p_init,R,covs,label,uwbpos);

% Simulate a random walk 
% Add noise to velocities and sensor measurements
theta=pi;
q=q_init;
ts=1;

% timesteps
ts_log=[0];
% true state log
q_log=[q];
% dead-reckoned state log
qdr_log=[q];
% EKF predicted state log
qekf_log=[q];
% measured state log
qm_log=[q];
% covariance log
p_log=[p_init];

figure(1)
hold on;
cutoff=100;
for i=1:cutoff
    % Randomly determine heading and calculate next q step with noise
    theta=theta+.5*(rand-0.5);
    q=q_log(:,i)+0.03*rot2(theta)*[1;0]+[1;1]*(rand-0.5)*.05;
    
    % Real velocity with noise (separately generated from position noise)
    v=(q-q_log(:,i))/ts+[1;1]*(rand-0.5)*.1;
    
    % Display random walk live
    %scatter([q(1,1)],[q(2,1)])
    %xlim([-10 10])
    %ylim([-10 10])
    %pause(.01);
    %disp(i);
    
    % stand-in UWB sensor state measurement with noise
    % TODO: maybe implement an actual intersection of spheres solver
    q_measured=q+[1;1]*(rand-0.5)*.5;
    
    [q_pred,p_pred]=uwb1_ekf.kalman_update(v,ts,q_measured,qekf_log(:,i),p_log(:,i*4-3:i*4));
    
    ts_log=[ts_log,ts_log(1,i)+ts];
    q_log=[q_log,q];
    qdr_log=[qdr_log,qdr_log(:,i)+v*ts];
    qekf_log=[qekf_log,q_pred(1:2,1)];
    qm_log=[qm_log,q_measured];
    p_log=[p_log,p_pred];
end

figure(2)
hold on;
plot(ts_log(1,1:cutoff),q_log(1,1:cutoff)');
plot(ts_log(1,1:cutoff),qdr_log(1,1:cutoff)');
plot(ts_log(1,1:cutoff),qm_log(1,1:cutoff)');
plot(ts_log(1,1:cutoff),qekf_log(1,1:cutoff)');
legend("q","qdr","qm","qekf");