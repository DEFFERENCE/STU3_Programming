ts = 0.001;
Pris_A_Kal = [0 1 0 0; 
    0 (-4*(pi^2)*Pris_b)/(End_Effector_Mass*(l^2)) -1 (2*pi*Pris_kt)/(End_Effector_Mass*l);
    0 0 0 0;
    0 -Pris_ke/Pris_Lm 0 -Pris_R/Pris_Lm];

Pris_B_Kal = [0; 0; 0; 1/Pris_Lm];
Pris_C_Kal = eye(4);  % or a specific output matrix depending on your system
Pris_D_Kal = zeros(4,1);  % assuming single input
Pris_sys_c = ss(Pris_A_Kal, Pris_B_Kal, Pris_C_Kal, Pris_D_Kal);
Pris_sys_d = c2d(Pris_sys_c, ts, 'tustin')


%% 
Rev_A_Kal = [0 1 0 0; 
    0 -Rev_b/Rev_J -1/Rev_J  Rev_kt/Rev_J;
    0 0 0 0;
    0 -Rev_ke/Rev_Lm 0 -Rev_R/Rev_Lm];

Rev_B_Kal = [0; 0; 0; 1/Rev_Lm];
Rev_C_Kal = eye(4);  % or a specific output matrix depending on your system
Rev_D_Kal = zeros(4,1);  % assuming single input
Rev_sys_c = ss(Rev_A_Kal, Rev_B_Kal, Rev_C_Kal, Rev_D_Kal);
Rev_sys_d = c2d(Rev_sys_c, ts, 'tustin')

%% 
