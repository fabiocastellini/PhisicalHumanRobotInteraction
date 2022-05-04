
%This function implements the Kalman Filter, with the option to compute
%the steady-state results

function [filter_pos, filter_vel, filter_acc, P_inf] = kalmanFilter(A,B,C,M_pos,R,Q,x_0,P_0,K_inf)
            
    K_1 = P_0*C.' * inv(C*P_0*C.' + R);  %initial Kalman G ain: K_{0+1}
    x_1 = A*x_0 + K_1 *(M_pos(1) - C*A*x_0); %initial optimal filter: xhat_{0+1|0+1}

    
    x_prev = x_1; %slides: x (pos+vel+acc) at time K given measurements until (K-1)
    P_prev = P_0; %slides: P at time K given measurements until (K-1)
    
    filter_pos = zeros(size(M_pos));
    filter_vel = zeros(size(M_pos));
    
    if size(x_0,1) == 3 %acceleration estimation is required!
        filter_acc = zeros(size(M_pos));
    else
        filter_acc = [];
    end
    
    for k=1:(size(M_pos,1)-1)       
        %Retrieve the "next" value of measured position
        y_next = M_pos(k+1); %slides: y at time (K+1)
        
        if nargin == 9 %steady state Kalman Filter      
            x_next = A*x_prev + K_inf*(y_next - C*A*x_prev);
            x_prev = x_next; %update the previous estimation with the current one      
        else                
            %Compute the "next" value of P (slides: P at time K+1 given measurements until K)            
            P_next = A*P_prev*A.' - A*P_prev*C.' * inv(C*P_prev*C.' + R) * C*P_prev*A.' + Q;
            P_prev = P_next; %update the previous P with the current one

            %Compute the "next" value of Kalman Gain (slides: K.Gain at time (K+1))
            K_next = P_next * C.' * inv(C*P_next*C.' + R);
        
            %Estimate the "next" value of x (slides: x at time (K+1) given measurements (K+1))
            x_next = A*x_prev + K_next*(y_next - C*A*x_prev);
            x_prev = x_next; %update the previous estimation with the current one
        end
        
        %Add the estimated position, velocity and acceleration to the appropriate arrays
        filter_pos(k) = x_next(1); filter_vel(k) = x_next(2); 
        if size(x_0,1) == 3 %acceleration estimation is required!
            filter_acc(k) = x_next(3);
        end
    end
    
    P_inf = P_prev; %return the limit for k->inf. of P(K|K-1)
    
    %Remove the last values because they're zeros
    %EDIT: instead of removing them, assign them to the previous number
%     filter_pos(end) = []; filter_vel(end) = []; 
    filter_pos(end) = filter_pos(end-1); filter_vel(end) = filter_vel(end-1); 

    if size(x_0,1) == 3 %acceleration estimation is required!
%         filter_acc(end) = [];
        filter_acc(end) = filter_acc(end-1);
    end
end