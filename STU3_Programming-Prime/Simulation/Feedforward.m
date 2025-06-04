%% Disturbance Feedforward : Revolute

ts = 0.001;
Rev_Dis_numerator = [Rev_Lm,Rev_R];
Rev_Dis_denominator = [Rev_Km*sampling_time,Rev_Km];
Rev_Dis_tf_s = tf(Rev_Dis_numerator ,Rev_Dis_denominator);
Rev_Dis_tf_Z = c2d(Rev_Dis_tf_s ,ts, 'tustin');
Rev_Dis_tf_Z.Variable = 'z^-1'
%% Refference Feedforward : Revolute

Rev_Ref_numerator = [Rev_Lm*I_static,I_static*Rev_R+Rev_b*Rev_Lm,Rev_Km*Rev_ke+Rev_b+Rev_R];
Rev_Ref_denominator = [Revo_Time_Constant^2,2*Revo_Time_Constant,1];
Rev_Ref_tf_s = tf(Rev_Ref_numerator ,Rev_Ref_denominator);
Rev_Ref_tf_Z = c2d(Rev_Ref_tf_s ,ts, 'tustin');
Rev_Ref_tf_Z.Variable = 'z^-1'

%% Disturbance Feedforward : Prismatic ///// อย่าลืมคูณค่า Gain

Pris_Dis_numerator = [Pris_Lm,Pris_R];
Pris_Dis_denominator = [Pris_Time_Constant,1];
Pris_Dis_tf_s = tf(Pris_Dis_numerator ,Pris_Dis_denominator);
Pris_Dis_tf_Z = c2d(Pris_Dis_tf_s ,ts, 'tustin');
Pris_Dis_tf_Z.Variable = 'z^-1'

%% Refference Feedforward : Revolute
lead = Lead*0.001;
Pris_Ref_numerator = [Pris_Lm*End_Effector_Mass*(lead)^2+2*pi*Pris_b*End_Effector_Mass*lead*Pris_Lm,Pris_R*End_Effector_Mass*lead^2+2*pi*Pris_b*End_Effector_Mass*lead *Pris_R,4*pi^2*Gear_ratio*Pris_Km*Pris_ke];
Pris_Ref_denominator = [Revo_Time_Constant^2,2*Revo_Time_Constant,1];
Pris_Ref_tf_s = tf(Pris_Ref_numerator ,Pris_Ref_denominator);
Pris_Ref_tf_Z = c2d(Pris_Ref_tf_s ,ts, 'tustin');
Pris_Ref_tf_Z.Variable = 'z^-1'
