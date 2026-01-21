%---------------------------------------------------------------------------
% Modellbildung und Simulation: GraphSLAM zur Schätzung von Roboterpositionen
% (x,y,th) sowie Umgebungsmerkmalen (x,y), wozu Abstände, Winkel und die 
% Roboterbewegung ausgewertet werden
% © Prof. Dr. Volker Sommer, Berliner Hochschule für Technik
%---------------------------------------------------------------------------
clear
clc
close all

% Bei dieser GraphSLAM-Variante werden die durch Bewegung bewirkten 
% Positionsänderungen von Zustand x_1 über x_2 zu x_3 erfasst. Als Messgrößen
% dienen die Abstände und Winkel von den Roboterpositionen (x,y,th) zu den 
% punktförmigen Umgebungsmerkmalen m_1 und m_2, die jeweils durch x und y 
% beschrieben werden und den Zuständen x_4 und x_5 entsprechen.
% Wegen des bezüglich der Winkel nichtlinearen Messmodells können nur
% inkrementelle Änderungen der Zustände durch Least-Squares berechnet
% werden, weshalb mehrere Schritte erforderlich sind. In jedem Schritt
% wird ein Vektor ze_ mit den erwarteten Messgrößen aus den jeweils aktuellen
% Schätzwerten der Zustände im Vektor x_ ermittelt, und der Differenzvektor delta_z_ 
% zu den tatsächlichen Messgrößen z_ gebildet. Aus delta_z_ wird dann mittels
% Least-Squares die Änderung des Zustandsvektors delta_x_ geschätzt und zu x_ addiert.
% Die Berechnung endet, sobald delta_x_ unterhalb einer Schwelle liegt.

% Startwerte der zu schätzenden Zustände vorgeben
xs_(1) =  0;             % geschätzte x-Koordinate der 1. Zustandes (Roboterposition 1)
xs_(2) =  0;             % geschätzte y-Koordinate der 1. Zustandes (Roboterposition 1)
xs_(3) =  0;             % geschätzte Ausrichtung der  1. Zustandes (Roboterposition 1)
xs_(4) =  1;             % geschätzte x-Koordinate der 2. Zustandes (Roboterposition 2)
xs_(5) =  0;             % geschätzte y-Koordinate der 2. Zustandes (Roboterposition 2)
xs_(6) =  0;             % geschätzte Ausrichtung der  2. Zustandes (Roboterposition 2)
xs_(7) =  1;             % geschätzte x-Koordinate der 3. Zustandes (Roboterposition 3)
xs_(8) = -1;             % geschätzte y-Koordinate der 3. Zustandes (Roboterposition 3)
xs_(9) = -90 * pi/180;   % geschätzte Ausrichtung der  3. Zustandes (Roboterposition 3)
xs_(10) =  0.5;          % geschätzte x-Koordinate des 4. Zustandes (Merkmal 1)
xs_(11) = -0.5;          % geschätzte y-Koordinate des 4. Zustandes (Merkmal 1)
xs_(12) =  0;            % geschätzte x-Koordinate des 5. Zustandes (Merkmal 2)
xs_(13) = -1;            % geschätzte y-Koordinate des 5. Zustandes (Merkmal 2)

x_ = xs_';   % In Spaltenvektor umwandeln

% Messvektor mit fixierten Koordinaten, Positionsänderungen, Abständen und Winkeln vorgeben
z_= [  0                % x1     
       0                % y1     
       0                % th1    
       1                % x12    
       0                % y12    
       0                % th12   
       0                % x23    
      -1                % y23    
      -90 * pi/180      % th23   
       1/sqrt(2)        % d14    
      -45 * pi/180      % phi14  
       1                % d15    
      -90 * pi/180      % phi15  
       1/sqrt(2)        % d24    
      -135 * pi/180     % phi24  
       sqrt(2)          % d25    
      -135 * pi/180     % phi25  
       1/sqrt(2)        % d34    
      -135 * pi/180     % phi34  
       1                % d35    
      -90 * pi/180 ];   % phi35  

% Zuverlässigkeit der erfassten Positionsänderungen und Messwerte in z_ vorgeben
% (ohne Berücksichtigung von Kovarianzen)
% Einträge in Si entsprechen den Kehrwerten der jeweiligen Varianz
Si = diag([1e6 1e6 1e6 1 1 0.1 1 1 0.1 1 0.1 1 0.1 1 0.1 1 0.1 1 0.1 1 0.1]); % Inverse Kovarianzmatrix vorgeben

% Rekursive Schätzung der Zustände, solange Fehler zu groß
e = 1;  % Anfangsfehler setzen
i = 0;  % Laufindex
while e > 1e-5

    % Aktuelle Werte der Zustände laden
    x1  = x_(1);
    y1  = x_(2);
    th1 = x_(3);
    x2  = x_(4);
    y2  = x_(5);
    th2 = x_(6);
    x3  = x_(7);
    y3  = x_(8);
    th3 = x_(9);
    x4 = x_(10);
    y4 = x_(11);
    x5 = x_(12);
    y5 = x_(13);

    % Erwartete Winkel von Roboterpositionen (x,y) zu Merkmalen berechnen
    alpha14 = atan2(y4-y1, x4-x1); % Winkel von x_1 auf x_4
    alpha15 = atan2(y5-y1, x5-x1); % Winkel von x_1 auf x_5
    alpha24 = atan2(y4-y2, x4-x2); % Winkel von x_2 auf x_4
    alpha25 = atan2(y5-y2, x5-x2); % Winkel von x_2 auf x_5
    alpha34 = atan2(y4-y3, x4-x3); % Winkel von x_3 auf x_4
    alpha35 = atan2(y5-y3, x5-x3); % Winkel von x_3 auf x_5

    % Cosinus- und Sinus-Werte der Verbindungsgeraden 
    % zwischen Roboterpositionen und Merkmalen bestimmen
    c14 = cos(alpha14);
    s14 = sin(alpha14);
    c15 = cos(alpha15);
    s15 = sin(alpha15);
    c24 = cos(alpha24);
    s24 = sin(alpha24);
    c25 = cos(alpha25);
    s25 = sin(alpha25);
    c34 = cos(alpha34);
    s34 = sin(alpha34);
    c35 = cos(alpha35);
    s35 = sin(alpha35);

    % Parameter zur Schätzung der Winkeländerungen bestimmen
    a14 = sin(alpha14)/d14;
    b14 = cos(alpha14)/d14;
    a15 = sin(alpha15)/d15;
    b15 = cos(alpha15)/d15;
    a24 = sin(alpha24)/d24;
    b24 = cos(alpha24)/d24;
    a25 = sin(alpha25)/d25;
    b25 = cos(alpha25)/d25;
    a34 = sin(alpha34)/d34;
    b34 = cos(alpha34)/d34;
    a35 = sin(alpha35)/d35;
    b35 = cos(alpha35)/d35;

    % Erwartete Messgrößen aus den Zuständen ermitteln
    % Hinweis: Die Funktion wrapToPi() normiert Winkel auf den Bereich von -pi bis pi,
    % damit die Differenz identischer Winkel null immer ergibt
    ze_= [ 0                        % x1
           0                        % y1
           0                        % th1
           x2-x1                    % x12
           y2-y1                    % y12
           wrapToPi(th2-th1)        % th12
           x3-x2                    % x23
           y3-y2                    % y23
           wrapToPi(th3-th2)        % th23
           c14*(x4-x1)+s14*(y4-y1)  % d14
           wrapToPi(alpha14-th1)    % phi14
           c15*(x5-x1)+s15*(y5-y1)  % d15
           wrapToPi(alpha15-th1)    % phi15
           c24*(x4-x2)+s24*(y4-y2)  % d24
           wrapToPi(alpha24-th2)    % phi24
           c25*(x5-x2)+s25*(y5-y2)  % d25
           wrapToPi(alpha25-th2)    % phi25
           c34*(x4-x3)+s34*(y4-y3)  % d34
           wrapToPi(alpha34-th3)    % phi34
           c35*(x5-x3)+s35*(y5-y3)  % d35
           wrapToPi(alpha35-th3) ]; % phi35

    delta_z_ = z_ - ze_    % Differenz zwischen erwarteten und tatsächlichen Messwerten

    % Festlegung der m x n Messmatrix H zur Bestimmung der Abweichungen von den aktuellen Schätzwerten
    H = [ 1    0    0    0    0    0    0    0    0    0    0    0    0     % zur Fixierung von delta_x1 = 0 
          0    1    0    0    0    0    0    0    0    0    0    0    0     % zur Fixierung von delta_y1 = 0
          0    0    1    0    0    0    0    0    0    0    0    0    0     % zur Fixierung von delta_th1 = 0
         -1    0    0    1    0    0    0    0    0    0    0    0    0     % für Schätzung von delta_x12
          0   -1    0    0    1    0    0    0    0    0    0    0    0     % für Schätzung von delta_y12
          0    0   -1    0    0    1    0    0    0    0    0    0    0     % für Schätzung von delta_th12
          0    0    0   -1    0    0    1    0    0    0    0    0    0     % für Schätzung von delta_x23
          0    0    0    0   -1    0    0    1    0    0    0    0    0     % für Schätzung von delta_y23
          0    0    0    0    0   -1    0    0    1    0    0    0    0     % für Schätzung von delta_th23
         -c14 -s14  0    0    0    0    0    0    0    c14  s14  0    0     % für Schätzung von delta_d14
          a14 -b14 -1    0    0    0    0    0    0   -a14  b14  0    0     % für Schätzung von delta_phi14
         -c15 -s15  0    0    0    0    0    0    0    0    0    c15  s15   % für Schätzung von delta_d15
          a15 -b15 -1    0    0    0    0    0    0    0    0   -a15  b15   % für Schätzung von delta_phi15
          0    0    0   -c24 -s24  0    0    0    0    c24  s24  0    0     % für Schätzung von delta_d24
          0    0    0    a24 -b24 -1    0    0    0   -a24  b24  0    0     % für Schätzung von delta_phi24
          0    0    0   -c25 -s25  0    0    0    0    0    0    c25  s25   % für Schätzung von delta_d25
          0    0    0    a25 -b25 -1    0    0    0    0    0   -a25  b25   % für Schätzung von delta_phi25
          0    0    0    0    0    0   -c34 -s34  0    c34  s34  0    0     % für Schätzung von delta_d34
          0    0    0    0    0    0    a34 -b34 -1   -a34  b34  0    0     % für Schätzung von delta_phi34
          0    0    0    0    0    0   -c35 -s35  0    0    0    c35  s35   % für Schätzung von delta_d35
          0    0    0    0    0    0    a35 -b35 -1    0    0   -a35  b35 ];% für Schätzung von delta_phi35
  
    % H*delta_x_ = delta_z_ ist nicht lösbar, löse daher H^T*H*delta_x_=H^T*delta_z_ (least square)
    delta_x_ = (H'*Si*H)\(H'*Si*delta_z_);

    x_ = x_ + delta_x_;   % geschätzten Zustandsvektor updaten

    % Betrag von delta_x_ im aktuellen Schritt für Abbruchkriterium berechnen
    e = sqrt(delta_x_'*delta_x_)
    i = i+1
    %pause
end
x_ - xs_'   % Differenz zwischen geschätzten Zuständen und deren Anfangswerten
Sx = inv(H'*Si*H)