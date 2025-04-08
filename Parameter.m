%% Revolute Motor

Rev_kt = 5.37E-02;
Rev_ke = 5.54E-02;
Rev_Eff = 9.70E-01;
Rev_Km = Rev_Eff* Rev_ke;
Rev_Lm = 3.05E-03;
Rev_R = 4.00E+00;
Rev_b = 1.06E-04;
Rev_J = 1.77E-05;

%% Prismatic Motor

Pris_kt = 5.37E-02;
Pris_ke = 5.54E-02;
Pris_Eff = 9.70E-01;
Pris_Km = Pris_Eff* Pris_ke;
Pris_Lm = 3.05E-03;
Pris_R = 4.00E+00;
Pris_b = 1.06E-04;
Pris_J = 1.77E-05;

%% Prismatic Axis

Prismatic_Intertia = 1; % kg*m^2
Prismatic_Mass = 1; % kg
Prismatic_Length = 1; % m

%% Revolute Axis


%% End Effector

End_Effector_Inertia = 0.1; % kg*m^2
End_Effector_Mass = 0.5; % kg

%% Ball_Screw

 Lead = 10 ; %mm

%% Other

g = 9.81 ; % m/s^2
Gear_ratio = 4 ;
sampling_time = 0.001;