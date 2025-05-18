%% Revolute Motor

Rev_kt = 5.37E-02;
Rev_ke = 1.63E+00;
Rev_Eff = 7.36E-01;
Rev_Km = Rev_Eff* Rev_ke;
Rev_Lm = 3.37E-04;
Rev_R = 5.13E-01;
Rev_b = 6.91E-03;
Rev_J = 1.88E-01;

%% Prismatic Motor

Pris_kt = 5.37E-02;
Pris_ke = 1.77E-01;
Pris_Eff = 8.75E-01;
Pris_Km = Pris_Eff* Pris_ke;
Pris_Lm = 0.0016;
Pris_R = 3.8719;
Pris_b = 4.82E-04;
Pris_J = 2.29E-04;

%% Prismatic Axis

Prismatic_Intertia = 6; % kg*m^2
Prismatic_Mass = 6; % kg
Prismatic_Length = 1; % m
Pris_Time_Constant = 0.01;

%% Revolute Axis
Revo_Time_Constant = 0.01 ;
I_static = Rev_J+Prismatic_Intertia+0.1;
%% End Effector

End_Effector_Inertia = 0.1; % kg*m^2
End_Effector_Mass = 0.5; % kg

%% Ball_Screw

 Lead = 10 ; %mm

%% Other

g = 9.81 ; % m/s^2
Gear_ratio = 4 ;
sampling_time = 0.001;
%% 

% คำนวณ numerator และ denominator แล้ว assign เป็น vector
num = [Pris_J * Pris_Lm, 
       Pris_b * Pris_Lm + Pris_R * Pris_J, 
       Pris_R * Pris_b + Pris_ke * Pris_Km];

den = [Pris_J * Pris_Time_Constant^3, 
       3 * Pris_J * Pris_Time_Constant^2 + Pris_b * Pris_Time_Constant^3, 
       3 * Pris_J * Pris_Time_Constant + 3 * Pris_b * Pris_Time_Constant^2, 
       Pris_J + 3 * Pris_b * Pris_Time_Constant, 
       Pris_b];
