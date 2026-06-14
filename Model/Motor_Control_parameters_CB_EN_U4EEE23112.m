%% PMSM Current Reference LUT Generator (with Power Limit & MTPA)
wMechanical = [0:100:12000]; % 100 RPM steps instead of 500
TqRef       = [-250:2:250];  % 2 Nm steps instead of 10
Vdc_vec     = [700, 750, 800];

% Motor Parameters
lambda = 0.05;
Ld     = 0.00015;
Lq     = 0.00045;
p      = 4;
P_max  = 104e3; % Maximum mechanical power in Watts (104 kW)

nW   = length(wMechanical);
nTq  = length(TqRef);
nVdc = length(Vdc_vec);

id_table = zeros(nW, nTq, nVdc);
iq_table = zeros(nW, nTq, nVdc);

% Suppress optimization output from fzero in the command window
options = optimset('Display','off');

for k = 1:nVdc
    Vdc = Vdc_vec(k);
    Vph = Vdc / sqrt(3);
    
    for i = 1:nW
        % Calculate both mechanical and electrical rad/s
        wm_rad = (wMechanical(i) / 60) * 2 * pi; 
        we = p * wm_rad;                         
        
        for j = 1:nTq
            Tq_req = TqRef(j);
            
            % --- 1. Apply Power Limit (Derate torque at high speeds) ---
            if wm_rad > 0
                Tq_limit = P_max / wm_rad;
                % Bound the requested torque to the power limit, maintaining sign
                Tq = sign(Tq_req) * min(abs(Tq_req), Tq_limit);
            else
                Tq = Tq_req;
            end
            
            % Catch zero torque condition early
            if Tq == 0
                id_table(i,j,k) = 0;
                iq_table(i,j,k) = 0;
                continue; 
            end
            
            % --- 2. Calculate MTPA Target Currents ---
            % For an IPMSM (Lq > Ld), Id must be negative to generate reluctance torque.
            % We solve for the Iq magnitude that produces the required Torque 
            % while constrained by the MTPA Id equation.
            
            % Define Torque error equation as a function of Iq magnitude
            torque_eq = @(Iq) 1.5 * p * Iq * (lambda + (Ld - Lq) * ...
                ((lambda - sqrt(lambda^2 + 8*(Lq-Ld)^2 * Iq^2)) / (4*(Lq-Ld)))) - abs(Tq);
            
            % Initial guess for Iq (using SPM assumption as starting point)
            iq_guess = abs(Tq) / (1.5 * p * lambda); 
            
            % Solve for optimal Iq magnitude using fzero
            iq_opt_mag = fzero(torque_eq, iq_guess, options);
            
            % Calculate corresponding optimal MTPA Id (will be negative)
            id_mtpa = (lambda - sqrt(lambda^2 + 8*(Lq-Ld)^2 * iq_opt_mag^2)) / (4*(Lq-Ld));
            iq_mtpa = sign(Tq) * iq_opt_mag; % Restore direction (motoring/regen)
            
            
            % --- 3. Apply Voltage Limits (Field Weakening check) ---
            if we == 0
                % At zero speed, voltage is not a constraint
                id_table(i,j,k) = id_mtpa;
                iq_table(i,j,k) = iq_mtpa;
            else
                % Check if the optimal MTPA point exceeds available phase voltage
                % V^2 = (we*Lq*Iq)^2 + (we*Ld*Id + we*lambda)^2
                v_backemf_d = -we * Lq * iq_mtpa;
                v_backemf_q = we * Ld * id_mtpa + we * lambda;
                v_mag_sq = v_backemf_d^2 + v_backemf_q^2;
                
                if v_mag_sq > Vph^2
                    % Voltage limit exceeded -> Enter Field Weakening (FW)
                    % Push Id further negative towards the center of the voltage ellipse
                    id_fw = (Vph/we - lambda) / Ld; 
                    
                    % Ensure Id_fw doesn't exceed the MTPA value or a safe max current limit (e.g., -500A)
                    id_fw = min(id_mtpa, max(-500, id_fw)); 
                    
                    id_table(i,j,k) = id_fw;
                    
                    % Calculate available Iq given the new, deeper negative Id_fw
                    val = Vph^2 - (we * Ld * id_fw + we * lambda)^2;
                    
                    if val > 0
                        iq_fw = sqrt(val) / (we * Lq);
                        iq_table(i,j,k) = sign(Tq) * min(abs(iq_mtpa), iq_fw);
                    else
                        % Deep field weakening: requested voltage is unreachable even at max negative Id
                        iq_table(i,j,k) = 0; 
                    end
                else
                    % Voltage limit not exceeded -> Use MTPA values
                    id_table(i,j,k) = id_mtpa;
                    iq_table(i,j,k) = iq_mtpa;
                end
            end
        end
    end
end

id_LUT = id_table;
iq_LUT = iq_table;
fprintf('Done. id_LUT size: %dx%dx%d\n', size(id_LUT,1), size(id_LUT,2), size(id_LUT,3));