
%This function implements the Kalman Predictor, with the option to compute
%the steady-state results

function [predictor_pos, predictor_vel, predictor_acc, P_inf] = kalmanPredictor(A,B,C,M_pos,R,Q,x_0,P_0,Kbar_inf)

    Kbar_0 = P_0*C.' * inv(C*P_0*C.' + R);      %initial Kalman G ain: Kbar_{0}
    x_1 = A*x_0 + Kbar_0 *(M_pos(1) - C*A*x_0); %initial optimal filter: xhat_{0+1|0}

    x_prev = x_1; %slides: x (pos+vel+acc) at time K given measurements until (K-1)
    P_prev = P_0; %slides: P at time K given measurements until (K-1)

    predictor_pos = zeros(size(M_pos));
    predictor_vel = zeros(size(M_pos));
    
    if size(x_0,1) == 3 %acceleration estimation is required!
        predictor_acc = zeros(size(M_pos));
    else
        predictor_acc = [];
    end
    
    for k=1:(size(M_pos,1)-1)
        %Retrieve the "next" value of measured position
        y_actual = M_pos(k); %slides: y at time (K+1)

        if nargin == 9 %steady state Kalman Predictor 
            %Estimate the "next" value of x (slides: x at time (K+1) given measurements until K)
            x_next = A*x_prev + Kbar_inf*(y_actual - C*x_prev);
            x_prev = x_next; %update the previous estimation with the current one
        else
            %Compute the "next" value of P (slides: P at time K given measurements until (K+1))
            P_next = A*P_prev*A.' - A*P_prev*C.' * inv(C*P_prev*C.' + R) * C*P_prev*A.' + Q;

            %Compute the "previous" value of Kalman Gain (slides: K.Gain at time K)
            Kbar_actual = A*P_prev * C.' * inv(C*P_prev*C.' + R);
            P_prev = P_next; %update the previous P with the current one

            %Estimate the "next" value of x (slides: x at time (K+1) given measurements until K)
            x_next = A*x_prev + Kbar_actual*(y_actual - C*x_prev);
            x_prev = x_next; %update the previous estimation with the current one
        end
        
        %Add the estimated position, velocity and acceleration to the appropriate arrays
        predictor_pos(k) = x_next(1); predictor_vel(k) = x_next(2);
        if size(x_0,1) == 3 %acceleration estimation is required!
            predictor_acc(k) = x_next(3);
        end
    end
    P_inf = P_prev;
    
        
    %Remove the last values because they're zeros
    %EDIT: instead of removing them, assign them to the previous number
%     filter_pos(end) = []; filter_vel(end) = []; 
    predictor_pos(end) = predictor_pos(end-1); predictor_vel(end) = predictor_vel(end-1); 

    if size(x_0,1) == 3 %acceleration estimation is required!
%         filter_acc(end) = [];
        predictor_acc(end) = predictor_acc(end-1);
    end

end