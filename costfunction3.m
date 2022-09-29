function cost3 = costfunction3(X_ref_3,x_k_3,u3)
    
% Definition of variables
Ts = 0.01;
g_bar = [20,20,20];
N1 = 10;


% state space model after time delay estimation 
    A_3 = [1 Ts 0 0;
        0 2 0 -1;
        1 0 0 0
        0 1 0 0];
    B_3 = [0; (g_bar(3)*Ts); 0; 0];
    [n, m] = size(B_3);
    
    % the basic idea, here, is prediction of the future state base on the
    % current information and system model 
    % the detail algorithm is presented in Dr. Prof. Daniel Görges's model
    % prediction control lectures, Technical university of kaiserslautern
    
    Phi_3 = zeros(N1*n,n); 
    Gamma_3 = zeros(N1*n,N1*m);
   
  % for joint 3 prediction update
        for r = 1:N1
    
        Phi_3(((r-1)*n)+1:r*n,1:n) = A_3^r;
    
            for c = 1:N1
                if r >= c
            
                Gamma_3(((r-1)*n)+1:r*n,((c-1)*m)+1:c*m) = A_3^(r-c)*B_3;
            
                end
            end
        end
    
        % predicted model
         X_k_3 = Phi_3*x_k_3 + Gamma_3*u3;
         
    % the basic idea, here, is input optimization that satisfy the minimum cost 
    % the detail algorithm is presented in Dr. Prof. Daniel Görges's model
    % prediction control lectures, Technical university of kaiserslautern
    
    Omega = zeros(N1*n,N1*n);
    Psi = zeros(N1*m,N1*m);
    
    Q = [20 0 0 0;
        0 10 0 0;
        0 0 2 0;
        0 0 0 1];
    R = 1;
    
    for r = 1:N1
                if r < N1
        
            Omega((r-1)*n+1:r*n,(r-1)*n+1:r*n) = Q;
        else
            
            Omega((r-1)*n+1:r*n,(r-1)*n+1:r*n) = Q;
        
        end
    
        Psi((r-1)*m+1:r*m,(r-1)*m+1:r*m) = R;
    
    end
        
    % optimization * objective function*
    
        cost3 = (X_k_3-X_ref_3)'*Omega*(X_k_3-X_ref_3) + u3'*Psi*u3;
   
end
