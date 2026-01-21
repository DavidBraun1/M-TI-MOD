%---------------------------------------------------------------------------
% Modellbildung und Simulation: GraphSLAM zur Schätzung von Roboterpositionen
% (x,y,th) sowie Umgebungsmerkmalen (d,phi), wozu Abstände, Winkel und die 
% Roboterbewegung ausgewertet werden
% © Prof. Dr. Volker Sommer, Berliner Hochschule für Technik
%---------------------------------------------------------------------------
clear
clc
close all

% Bei dieser GraphSLAM-Variante werden die durch Bewegung bewirkten 
% Positionsänderungen von Zustand x_1 über x_2 zu x_3 erfasst. Als Messgrößen
% dienen die Abstände und Winkel von den Roboterpositionen (x,y,th) zu den
% Wänden m_1 und m_2, die jeweils durch d und phi beschrieben werden und den 
% Zuständen x_4 und x_5 im Zustandsvektor x_ entsprechen.
% Wegen des bezüglich der Normalenabstände nichtlinearen Messmodells können nur
% inkrementelle Änderungen der Zustände durch Least-Squares berechnet
% werden, weshalb mehrere Schritte erforderlich sind. In jedem Schritt
% wird ein Vektor ze_ mit den erwarteten Messgrößen aus den jeweils aktuellen
% Schätzwerten der Zustände im Vektor x_ ermittelt, und der Differenzvektor 
% delta_z_ zu den tatsächlichen Messgrößen z_ gebildet. Aus delta_z_ wird dann mittels
% Least-Squares die Änderung des Zustandsvektors delta_x_ geschätzt und zu x_ addiert.
% Die Berechnung endet, sobald delta_x_ unterhalb einer Schwelle liegt.

% Startwerte der zu schätzenden Zustände vorgeben (mit Fehlern)
xs_(1) =  0;             % geschätzte x-Koordinate der 1. Zustandes (Roboterposition 1)
xs_(2) =  0;             % geschätzte y-Koordinate der 1. Zustandes (Roboterposition 1)
xs_(3) =  0;             % geschätzte Ausrichtung der  1. Zustandes (Roboterposition 1)
xs_(4) =  1;             % geschätzte x-Koordinate der 2. Zustandes (Roboterposition 2)
xs_(5) =  0;             % geschätzte y-Koordinate der 2. Zustandes (Roboterposition 2)
xs_(6) =  0;             % geschätzte Ausrichtung der  2. Zustandes (Roboterposition 2)
xs_(7) =  1;             % geschätzte x-Koordinate der 3. Zustandes (Roboterposition 3)
xs_(8) = -1;             % geschätzte y-Koordinate der 3. Zustandes (Roboterposition 3)
xs_(9) = -90 * pi/180;   % geschätzte Ausrichtung der  3. Zustandes (Roboterposition 3)
xs_(10) =  0.6;          % geschätzter Normalenabstand des 4. Zustandes (Wand1 = Merkmal 1)
xs_(11) =  120 * pi/180; % geschätzter Normalenwinkel des 4. Zustandes (Wand1 = Merkmal 1)
xs_(12) =  sqrt(3)/2+0.4;% geschätzter Normalenabstand des 5. Zustandes (Wand2 = Merkmal 2)
xs_(13) =  30 * pi/180;  % geschätzter Normalenwinkel des 5. Zustandes (Wand2 = Merkmal 2)

% Exakte Zustände
% xs_(1) =  0;             % geschätzte x-Koordinate der 1. Zustandes (Roboterposition 1)
% xs_(2) =  0;             % geschätzte y-Koordinate der 1. Zustandes (Roboterposition 1)
% xs_(3) =  0;             % geschätzte Ausrichtung der  1. Zustandes (Roboterposition 1)
% xs_(4) =  1;             % geschätzte x-Koordinate der 2. Zustandes (Roboterposition 2)
% xs_(5) =  0;             % geschätzte y-Koordinate der 2. Zustandes (Roboterposition 2)
% xs_(6) =  0;         % geschätzte Ausrichtung der  2. Zustandes (Roboterposition 2)
% xs_(7) =  1;             % geschätzte x-Koordinate der 3. Zustandes (Roboterposition 3)
% xs_(8) = -1;             % geschätzte y-Koordinate der 3. Zustandes (Roboterposition 3)
% xs_(9) = -90 * pi/180;   % geschätzte Ausrichtung der  3. Zustandes (Roboterposition 3)
% xs_(10) =  0.6;          % geschätzter Normalenabstand des 4. Zustandes (Wand1 = Merkmal 1)
% xs_(11) =  120 * pi/180; % geschätzter Normalenwinkel des 4. Zustandes (Wand1 = Merkmal 1)
% xs_(12) =  sqrt(3)/2+0.4;% geschätzter Normalenabstand des 5. Zustandes (Wand2 = Merkmal 2)
% xs_(13) =  30 * pi/180;  % geschätzter Normalenwinkel des 5. Zustandes (Wand2 = Merkmal 2)


x_ = xs_';   % In Spaltenvektor umwandeln

% Messvektor mit fixierten Koordinaten, Positionsänderungen, Abständen und Winkeln
z_= [  0                % x1     
       0                % y1     
       0                % th1    
       1                % x12    
       0                % y12    
       0                % th12   
       0                % x23    
      -1                % y23    
      -90 * pi/180      % th23   
       0.6              % d14   
       120 * pi/180     % phi14  
       sqrt(3)/2+0.4    % d15    
       30 * pi/180      % phi15  
       1.1              % d24    
       120 * pi/180     % phi24  
       0.4              % d25
       30 * pi/180      % phi25  
       sqrt(3)/2+1.1    % d34    
      -150 * pi/180     % phi34  
       0.9              % d35    
       120 * pi/180 ];  % phi35  

% Zuverlässigkeit der erfassten Positionsänderungen und Messwerte
% in z_ vorgeben (noch ohne Berücksichtigung von Kovarianzen)
% Einträge in Si entsprechen den Kehrwerten der jeweiligen Varianz
Si = eye(21);   % Default: Alle Messwerte erhalten dasselbe Gewicht 1

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
    d4 = x_(10);
    phi4 = x_(11);
    d5 = x_(12);
    phi5 = x_(13);

    % Abstände der Roboterpositionen vom Nullpunkt 
    % jeweils in Richtung der beiden Wandnormalen berechnen
    x14 = x1*cos(phi4) + y1*sin(phi4);
    x15 = x1*cos(phi5) + y1*sin(phi5);
    x24 = x2*cos(phi4) + y2*sin(phi4);
    x25 = x2*cos(phi5) + y2*sin(phi5);
    x34 = x3*cos(phi4) + y3*sin(phi4);
    x35 = x3*cos(phi5) + y3*sin(phi5);
    
    % Davon abhängig Vorzeichen festlegen
    V14 = 1;
    if x14 < d4     
        V14 = -1;
    end
    V15 = 1;
    if x15 < d5     
        V15 = -1;
    end
    V24 = 1;
    if x24 < d4     
        V24 = -1;
    end
    V25 = 1;
    if x25 < d5     
        V25 = -1;
    end
    V34 = 1;
    if x34 < d4     
        V34 = -1;
    end
    V35 = 1;
    if x35 < d5     
        V35 = -1;
    end

    % Davon abhängig Parameter zur Berechnung der gemessenen Abstände aus Zuständen bestimmen
    a14 = V14*cos(phi4);    % Abhängigkeit zwischen d14 und x1
    b14 = V14*sin(phi4);    % Abhängigkeit zwischen d14 und y1
    c14 = V14*(y1*cos(phi4)-x1*sin(phi4));  % Abhängigkeit zwischen d14 und phi4
    a15 = V15*cos(phi5);    % Abhängigkeit zwischen d15 und x1
    b15 = V15*sin(phi5);    % Abhängigkeit zwischen d15 und y1
    c15 = V15*(y1*cos(phi5)-x1*sin(phi5));  % Abhängigkeit zwischen d15 und phi5
    a24 = V24*cos(phi4);    % Abhängigkeit zwischen d24 und x2
    b24 = V24*sin(phi4);    % Abhängigkeit zwischen d24 und y2
    c24 = V24*(y2*cos(phi4)-x2*sin(phi4));  % Abhängigkeit zwischen d24 und phi4
    a25 = V25*cos(phi5);    % Abhängigkeit zwischen d25 und x2
    b25 = V25*sin(phi5);    % Abhängigkeit zwischen d25 und y2
    c25 = V25*(y2*cos(phi5)-x2*sin(phi5));  % Abhängigkeit zwischen d25 und phi5
    a34 = V34*cos(phi4);    % Abhängigkeit zwischen d34 und x3
    b34 = V34*sin(phi4);    % Abhängigkeit zwischen d34 und y3
    c34 = V34*(y3*cos(phi4)-x3*sin(phi4));  % Abhängigkeit zwischen d34 und phi4
    a35 = V35*cos(phi5);    % Abhängigkeit zwischen d35 und x3
    b35 = V35*sin(phi5);    % Abhängigkeit zwischen d35 und y3
    c35 = V35*(y3*cos(phi5)-x3*sin(phi5));  % Abhängigkeit zwischen d35 und phi5
   
    % Erwartete Messgrößen aus den Zuständen ermitteln
    % Hinweis: Die Funktion wrapToPi() normiert Winkel auf den Bereich von -pi bis pi,
    % damit die Differenz identischer Winkel null immer ergibt
    ze_= [ 0                                 % x1
           0                                 % y1
           0                                 % th1
           x2-x1                             % x12
           y2-y1                             % y12
           wrapToPi(th2-th1)                 % th12
           x3-x2                             % x23
           y3-y2                             % y23
           wrapToPi(th3-th2)                 % th23
           d4+V14*x14                        % d14
           wrapToPi(phi4-th1+pi*(V14+1)/2)   % phi14
           d5+V15*x15                        % d15
           wrapToPi(phi5-th1+pi*(V15+1)/2)   % phi15
           d4+V24*x24                        % d24
           wrapToPi(phi4-th2+pi*(V24+1)/2)   % phi24
           d5+V25*x25                        % d25
           wrapToPi(phi5-th2+pi*(V25+1)/2)   % phi25
           d4+V34*x34                        % d34
           wrapToPi(phi4-th3+pi*(V34+1)/2)   % phi34
           d5+V35*x35                        % d35
           wrapToPi(phi5-th3+pi*(V35+1)/2) ] % phi35

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
          a14  b14  0    0    0    0    0    0    0    1    c14  0    0     % für Schätzung von delta_d14
          0    0   -1    0    0    0    0    0    0    0    1    0    0     % für Schätzung von delta_phi14
          a15  b15  0    0    0    0    0    0    0    0    0    1    c15   % für Schätzung von delta_d15
          0    0    1    0    0    0    0    0    0    0    0    0    1     % für Schätzung von delta_phi15
          0    0    0    a24  b24  0    0    0    0    1    c24  0    0     % für Schätzung von delta_d24
          0    0    0    0    0   -1    0    0    0    0    1    0    0     % für Schätzung von delta_phi24
          0    0    0    a25  b25  0    0    0    0    0    0    1    c25   % für Schätzung von delta_d25
          0    0    0    0    0   -1    0    0    0    0    0    0    1     % für Schätzung von delta_phi25
          0    0    0    0    0    0    a34  b34  0    1    c34  0    0     % für Schätzung von delta_d34
          0    0    0    0    0    0    0    0   -1    0    1    0    0     % für Schätzung von delta_phi34
          0    0    0    0    0    0    a35  b35  0    0    0    1    c35   % für Schätzung von delta_d35
          0    0    0    0    0    0    0    0   -1    0    0    0    1  ]; % für Schätzung von delta_phi35

    % H*delta_x_ = delta_z_ ist nicht lösbar, löse daher H^T*H*delta_x_=H^T*delta_z_ (least square)
    delta_x_ = (H'*Si*H)\(H'*Si*delta_z_)

    x_ = x_ + delta_x_;   % geschätzten Zustandsvektor updaten

    % Betrag von delta_x_ im aktuellen Schritt für Abbruchkriterium berechnen
    e = sqrt(delta_x_'*delta_x_)
    i = i+1
    %pause
end
x_ - xs_'   % Differenz zwischen geschätzten Zuständen und deren Anfangswerten
