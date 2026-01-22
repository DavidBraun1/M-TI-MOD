clear

%-----------a-----------
%Startposition
x1_ = [0 0 0]';
x2_ = [1 0 0]';
x3_ = [2 0 0]';
x4_ = [0 -1]';       %geschätzt
x5_ = [2 -1]';       %geschätzt

%Bewegungen
x12_ = [1 0 0]';
x23_ = [1 0 0]';

%Messungen
d14 = 1.05;
d15 = 2.2;
d24 = 1.45;
d25 = 1.4;
d34 = 2.25;
d35 = 0.99;

%Bewegungs und Messdaten
z_ = [x1_; x12_; x23_; d14; d15; d24; d25; d34; d35];

%Schätzvektor
x_ = [x1_; x2_; x3_; x4_; x5_;];

%-----------b-----------
%invertierte Kovarianzmatrix festlegen
Sigma = diag([1e-6 1e-6 1e-6 1 1 1 1 1 1 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2]);
%            |  x1_         | x12_| x23_|           d14 - d35         |

Sigma_inv = inv(Sigma);

e = 1;
i = 0;

while e > 1e-5
    %-----------c-----------
    %Winkel zwischen Roboter und Merkmalen
    a14 = atan2(x4_(2) - x1_(2), x4_(1) - x1_(1));
    a15 = atan2(x5_(2) - x1_(2), x5_(1) - x1_(1));
    a24 = atan2(x4_(2) - x2_(2), x4_(1) - x2_(1));
    a25 = atan2(x5_(2) - x2_(2), x5_(1) - x2_(1));
    a34 = atan2(x4_(2) - x3_(2), x4_(1) - x3_(1));
    a35 = atan2(x5_(2) - x3_(2), x5_(1) - x3_(1));
    
    %Sinus und Cosinus ausrechnen
    c14 = cos(a14);
    s14 = sin(a14);
    c15 = cos(a15);
    s15 = sin(a15);
    c24 = cos(a24);
    s24 = sin(a24);
    c25 = cos(a25);
    s25 = sin(a25);
    c34 = cos(a34);
    s34 = sin(a34);
    c35 = cos(a35);
    s35 = sin(a35);
    
    %Zeilen ist z_ und Spalten ist x_
    %   |   x1_   |   x2_     |   x3_     |  x4_  |  x5_
    H = [...
        1   0   0   0   0   0   0   0   0   0   0   0   0   %x1_: x
        0   1   0   0   0   0   0   0   0   0   0   0   0   %x1_: y
        0   0   1   0   0   0   0   0   0   0   0   0   0   %x1_: theta
       -1   0   0   1   0   0   0   0   0   0   0   0   0   %x12_: x
        0  -1   0   0   1   0   0   0   0   0   0   0   0   %x12_: y
        0   0  -1   0   0   1   0   0   0   0   0   0   0   %x12_: theta
        0   0   0  -1   0   0   1   0   0   0   0   0   0   %x23_: x
        0   0   0   0  -1   0   0   1   0   0   0   0   0   %x23_: y
        0   0   0   0   0  -1   0   0   1   0   0   0   0   %x23_: theta
      -c14 -s14 0   0   0   0   0   0   0  c14 s14  0   0   %d14
      -c15 -s15 0   0   0   0   0   0   0   0   0  c15 s15  %d15
        0   0   0 -c24 -s24 0   0   0   0  c24 s24  0   0   %d24
        0   0   0 -c25 -s25 0   0   0   0   0   0  c25 s25  %d25
        0   0   0   0   0   0 -c34 -s34 0  c34 s34  0   0   %d34
        0   0   0   0   0   0 -c35 -s35 0   0   0  c35 s35  %d35
        ];

    %Erwartete Messgrößen aus den Zuständen ermitteln
    ze_ = [ 0; 0; 0;                                    %x1
            x_(4)-x_(1); x_(5)-x_(2); x_(6)-x_(3);      %x2
            x_(7)-x_(4); x_(8)-x_(5); x_(9)-x_(6);      %x3
            c14*(x_(10)-x_(1))+s14*(x_(11)-x_(2));      %d14
            c15*(x_(12)-x_(1))+s15*(x_(13)-x_(2));      %d15
            c24*(x_(10)-x_(4))+s24*(x_(11)-x_(5));      %d24
            c25*(x_(12)-x_(4))+s25*(x_(13)-x_(5));      %d25
            c34*(x_(10)-x_(7))+s34*(x_(11)-x_(8));      %d34
            c35*(x_(12)-x_(7))+s35*(x_(13)-x_(8));      %d35
        ];

    delta_z_ = z_ - ze_;
    delta_x_ = (H'*Sigma_inv*H)\(H'*Sigma_inv*delta_z_);
    x_ = x_ + delta_x_;
    
    %Abweichung
    e = sqrt(delta_x_'*delta_x_);
    i = i+1;
end
