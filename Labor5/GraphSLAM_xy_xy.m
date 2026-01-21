%--------------------------------------------------------------------------
% Modellbildung und Simulation: GraphSLAM zur Schätzung von Roboterpositionen
% (x,y) sowie Umgebungsmerkmalen (x,y) nur durch Messung von Abständen
% © Prof. Dr. Volker Sommer, Berliner Hochschule für Technik
%--------------------------------------------------------------------------
clear
clc
close all

% Bei dieser GraphSLAM-Variante werden nur die Abstände zwischen den zu
% schätzenden Zuständen im Vektor x_ als Messwerte im Vektor z_ verwendet. 
% Das Modell ist linear, allerdings müssen mehrere Iterationen mit den jeweils
% aktuell geschätzten Zuständen durchgeführt werden, da die für die Schätzung 
% benötigten Winkel zu Beginn nicht bekannt sind und daher zunächst aus den 
% Anfangswerten von x_ näherungsweise berechnet werden.

% Startwerte der zu schätzenden Zustände vorgeben
xs_(1) =  0;      % geschätzte x-Koordinate des 1. Zustandes
xs_(2) =  0;      % geschätzte y-Koordinate des 1. Zustandes
xs_(3) =  1;      % geschätzte x-Koordinate des 2. Zustandes
xs_(4) =  0;      % geschätzte y-Koordinate des 2. Zustandes
xs_(5) =  1;      % geschätzte x-Koordinate des 3. Zustandes
xs_(6) = -1;      % geschätzte y-Koordinate des 3. Zustandes
xs_(7) =  0.5;    % geschätzte x-Koordinate des 4. Zustandes (Merkmal 1)
xs_(8) = -0.5;    % geschätzte y-Koordinate des 4. Zustandes (Merkmal 1)
xs_(9) =  0;      % geschätzte x-Koordinate des 5. Zustandes (Merkmal 2)
xs_(10)= -1;      % geschätzte y-Koordinate des 5. Zustandes (Merkmal 2)

x_ = xs_';    % In Spaltenvektor umwandeln

% Fixierte Koordinaten und gemessene Abstände in Messvektor z_ eintragen
z_ = [  0           % x1
        0           % y1
        0           % y2
        1           % d12
        1/sqrt(2)   % d14
        1           % d15
        1           % d23
        1/sqrt(2)   % d24
        sqrt(2)     % d25
        1/sqrt(2)   % d34
        1       ];  % d35

% Inverse Kovarianzmatrix mit der Zuverlässigkeit der Messwerte in z_ vorgeben 
% (Einträge entsprechen dem Kehrwert der jeweiligen Varianz)
Si = [ 1e6  0  0  0  0  0  0  0  0  0  0
       0  1e6  0  0  0  0  0  0  0  0  0
       0  0  1e6  0  0  0  0  0  0  0  0
       0  0  0  1  0  0  0  0  0  0  0
       0  0  0  0  1  0  0  0  0  0  0
       0  0  0  0  0  1  0  0  0  0  0
       0  0  0  0  0  0  1  0  0  0  0
       0  0  0  0  0  0  0  1  0  0  0
       0  0  0  0  0  0  0  0  1  0  0
       0  0  0  0  0  0  0  0  0  1  0
       0  0  0  0  0  0  0  0  0  0  1];

e = 1; % Fehler dient als Abbruchkriterium
i = 0; % Laufindex

% Neue Schätzwerte für die Zustände werden iterativ in einer Schleife berechnet
while e > 1e-5

    % Aktuelle Zustände auslesen
    x1 = x_(1);
    y1 = x_(2);
    x2 = x_(3);
    y2 = x_(4);
    x3 = x_(5);
    y3 = x_(6);
    x4 = x_(7);
    y4 = x_(8);
    x5 = x_(9);
    y5 = x_(10);

    % Winkel der Verbindungslinien aus aktuellen Zuständen berechnen
    alpha12 = atan2(y2-y1, x2-x1);
    alpha14 = atan2(y4-y1, x4-x1);
    alpha15 = atan2(y5-y1, x5-x1);
    alpha23 = atan2(y3-y2, x3-x2);
    alpha24 = atan2(y4-y2, x4-x2);
    alpha25 = atan2(y5-y2, x5-x2);
    alpha34 = atan2(y4-y3, x4-x3);
    alpha35 = atan2(y5-y3, x5-x3);
    
    % Ausgabe der aktuellen Winkel
    disp(['alpha_12 = ', num2str(alpha12*180/pi)]);
    disp(['alpha_14 = ', num2str(alpha14*180/pi)]);
    disp(['alpha_15 = ', num2str(alpha15*180/pi)]);
    disp(['alpha_23 = ', num2str(alpha23*180/pi)]);
    disp(['alpha_24 = ', num2str(alpha24*180/pi)]);
    disp(['alpha_25 = ', num2str(alpha25*180/pi)]);
    disp(['alpha_34 = ', num2str(alpha34*180/pi)]);
    disp(['alpha_35 = ', num2str(alpha35*180/pi)]);

    % Daraus cos- und sin-Werte ermitteln, um die erwarteten Abstände zu bestimmen
    c12 = cos(alpha12);
    s12 = sin(alpha12);
    c14 = cos(alpha14);
    s14 = sin(alpha14);
    c15 = cos(alpha15);
    s15 = sin(alpha15);
    c23 = cos(alpha23);
    s23 = sin(alpha23);
    c24 = cos(alpha24);
    s24 = sin(alpha24);
    c25 = cos(alpha25);
    s25 = sin(alpha25);
    c34 = cos(alpha34);
    s34 = sin(alpha34);
    c35 = cos(alpha35);
    s35 = sin(alpha35);

    % Festlegung der globalen m x n Messmatrix H (hier: m=11, n=10)
    H = [ 1    0    0    0    0    0    0    0    0    0    % Zur Fixierung von x1 
          0    1    0    0    0    0    0    0    0    0    % Zur Fixierung von y1
          0    0    0    1    0    0    0    0    0    0    % Zur Fixierung von y2
         -c12 -s12  c12  s12  0    0    0    0    0    0    % für Abstand zwischen x_1 und x_2
         -c14 -s14  0    0    0    0    c14  s14  0    0    % für Abstand zwischen x_1 und x_4
         -c15 -s15  0    0    0    0    0    0    c15  s15  % für Abstand zwischen x_1 und x_5
          0    0   -c23 -s23  c23  s23  0    0    0    0    % für Abstand zwischen x_2 und x_3
          0    0   -c24 -s24  0    0    c24  s24  0    0    % für Abstand zwischen x_2 und x_4
          0    0   -c25 -s25  0    0    0    0    c25  s25  % für Abstand zwischen x_2 und x_5
          0    0    0    0   -c34 -s34  c34  s34  0    0    % für Abstand zwischen x_3 und x_4
          0    0    0    0   -c35 -s35  0    0    c35  s35];% für Abstand zwischen x_3 und x_5

    % H*x = z ist nicht lösbar, löse daher H^T*H*x=H^T*z (Least Squares)
    x_neu = (H'*Si*H)\(H'*Si*z_) 

    % Abweichung zur vorherigen Schätzung berechnen (dient als Abbruchkriterium)
    e = sqrt((x_-x_neu)' * (x_-x_neu))
    
    x_ = x_neu;
    i = i+1
    pause
end
x_ - xs_'   % Differenz zwischen geschätzten Zuständen und deren Anfangswerten
