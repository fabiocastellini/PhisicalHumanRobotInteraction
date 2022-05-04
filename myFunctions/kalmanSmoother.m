
%This function implements the Kalman Smoother

function [smoother_pos, smoother_vel, smoother_acc] = kalmanSmoother(A,B,C,M_pos,R,Q,x_0,P_0)
    
    %%- Kalman Filter (forward step) -%%
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
               
        %Compute P_{K|K} --> required for backward
        P_kk_all(:,:,k) = P_prev - P_prev*C.'*inv(C*P_prev*C.' + R) * C*P_prev;
        %Compute P_{K+1|K} --> required for backward
        P_kplus1_k_all(:,:,k) = A * P_kk_all(:,:,k) * A.' + Q;
        
        %Compute the "next" value of P (slides: P at time K given measurements until (K+1))
        P_next = A*P_prev*A.' - A*P_prev*C.' * inv(C*P_prev*C.' + R) * C*P_prev*A.' + Q;
        P_prev = P_next; %update the previous P with the current one

        %Compute the "next" value of Kalman Gain (slides: K.Gain at time (K+1))
        K_next = P_next * C.' * inv(C*P_next*C.' + R);

        %Estimate the "next" value of x (slides: x at time (K+1) given measurements (K+1))
        x_next = A*x_prev + K_next*(y_next - C*A*x_prev);
        x_prev = x_next; %update the previous estimation with the current one
    

        %Add the estimated position, velocity and acceleration to the appropriate arrays
        filter_pos(k) = x_next(1); filter_vel(k) = x_next(2); 
        if size(x_0,1) == 3 %acceleration estimation is required!
            filter_acc(k) = x_next(3);
        end
    end


    %%- Smoothing (backward step) --%
    if size(x_0,1) == 3 %acceleration estimation is required!
        xf = [filter_pos.';
              filter_vel.';
              filter_acc.'];
        xf(:,end) = [];
    else
        xf = [filter_pos.';
              filter_vel.'];
        xf(:,end) = [];
    end
    
    xs = zeros(size(xf));
    xs(:,end) = xf(:,end);
        
    for k=(size(xf,2)-1):-1:1
        Kcurve_k = P_kk_all(:,:,k) * A.' * inv(P_kplus1_k_all(:,:,k));
        xs(:,k) = xf(:,k) + Kcurve_k*(xs(:,k+1) - xf(:,k+1));
    end
    
    smoother_pos = xs(1,:).';
    smoother_vel = xs(2,:).';
    if size(x_0,1) == 3 %acceleration estimation is required!
        smoother_acc = xs(3,:).';
    else
        smoother_acc = [];
    end   
    
    
    smoother_pos(end+1) = smoother_pos(end); smoother_vel(end+1) = smoother_vel(end); 
    if size(x_0,1) == 3 %acceleration estimation is required!
        smoother_acc(end+1) = smoother_acc(end);
    end

end