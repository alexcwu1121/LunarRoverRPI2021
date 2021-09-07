% ekf.m
% Extended Kalman Filter

classdef ekf < handle
    
    properties (Access=private)
        covs
        R
        h
        h_trans
    end
    
    properties (Access=public)
        % Just placeholders to keep q and cov states. Matlab sucks
        q
        p
        label
    end
    
    methods
        function obj = ekf(q,p,R,covs,label,uwbpos)
            % Just placeholders
            obj.q=q;
            obj.p=p;
            % Process covariance
            obj.covs=covs;
            % Measurement covariance
            obj.R=R;
            obj.label=label;
            % Take positions of beacons and generate observation matrix and
            % observation jacobian
            [obj.h, obj.h_trans]=trilateration_transform(uwbpos);
        end
        
        function q_pred = predict(obj,v,ts,q_prev)
            q_pred=[1 0 ts 0;
                0 1 0 ts;
                0 0 1 0;
                0 0 0 1]*[q_prev;v];
        end
        
        function [q_pred,p_pred] = kalman_update(obj,v,ts,q_measured,q_prev,p_prev)
            syms x y
            
            % Get q prediction via dead reckoning
            q_pred = obj.predict(v,ts,q_prev);
            
            % Get covariance prediction via dead reckoning
            p_pred = [1 0 ts 0;
                0 1 0 ts;
                0 0 1 0;
                0 0 0 1]*p_prev*[1 0 ts 0;
                0 1 0 ts;
                0 0 1 0;
                0 0 0 1]'+obj.covs;
            
            % Apply trilateration transformation to derive range space
            % state and measurement residual
            z_pred = subs(obj.h,[x y],q_pred(1:2,1)');
            z_measured = subs(obj.h,[x y],q_measured');
            
            y_k = z_measured-z_pred;
            
            % Covariance residual
            h_trans_subs=eval(subs(obj.h_trans,[x y],q_pred(1:2,1)'))+eps;
            %h_trans_subs=eval(subs(obj.h_trans,[x y],q_prev(1:2,1)'));
            s_k = h_trans_subs*p_pred*h_trans_subs'+obj.R;
            
            % Compute Kalman gain
            K = p_pred*(h_trans_subs')/s_k;
            
            % Update state and covariance
            q_pred = eval(q_pred+K*y_k);
            p_pred = (eye(4)-K*h_trans_subs)*p_pred;
        end
    end
end