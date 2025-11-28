clear

%Addition von 2 Quaternionen
function out = qadd(x, y)
    out = x(:) + y(:);
end

%Betrag, also Länge eines Quaternion
function out = qnorm(x)
    out = sqrt(sum(x.^2));
end

%Konjugiertes Quaternion
function out = qconj(x)
    out = [x(1), x(2:4) .* -1];
end

%Skalare Multiplikation eines Quaternion
function out = qsmult(a, x)
    out = a .* x;
end

%Quaternion normalisieren auf den Betrag 1
function out = qnormalize(x)
    out = qsmult(1/qnorm(x), x);
end

%Multiplikation zweier Quaternionen nach Python Skript
function out = qmult(x, y)
    P = [0 0 0 0];
    for i=1:4
        for j=1:4
            if (i-1)*(j-1)== 0 || (i-1) == (j-1)
                if (i-1)*(j-1) == 0 || mod((i-1), 3) + 1 == (j-1)
                    P(abs((i-1)-(j-1))+1) = ...
                        P(abs((i-1)-(j-1))+1) + x(i) * y(j) * 1;
                else
                    P(abs((i-1)-(j-1))+1) = ...
                        P(abs((i-1)-(j-1))+1) + x(i) * y(j) * -1;
                end
            else
                if (i-1)*(j-1) == 0 || mod((i-1), 3) + 1 == (j-1)
                    P(abs(6-(i-1)-(j-1))+1) = ...
                        P(abs(6-(i-1)-(j-1))+1) + x(i) * y(j) * 1;
                else
                    P(abs(6-(i-1)-(j-1))+1) = ...
                        P(abs(6-(i-1)-(j-1))+1) + x(i) * y(j) * -1;
                end
            end
        end
    end
    out = P;
end

%Multiplikation zweier Quaternionen nach Matrixmultiplikation
function out = qmult1(x,y)
    out = [x(1) -x(2) -x(3) -x(4);
           x(2)  x(1) -x(4)  x(3);
           x(3)  x(4)  x(1) -x(2);
           x(4) -x(3)  x(2)  x(1)] * y(:);
end

function out = qrotate(v, u, phi)
    theta = (phi/180)*pi;
    Q(1) = cos(theta/2);
    Q(2:4) = qsmult(sin(theta/2), qnormalize(u));
    temp = qmult(qmult1(Q, [0, v]), qconj(Q));
    out = round(temp(2:4), 6);
    out(abs(out) < 1e-12) = 0;
end

function [out, Q, alpha, beta, gamma] = qrotate_plus(v, u, phi)
    theta = (phi/180)*pi;
    Q(1) = cos(theta/2);
    Q(2:4) = qsmult(sin(theta/2), qnormalize(u));
    Q(abs(Q) < 1e-12) = 0;
    temp = qmult(qmult1(Q, [0, v]), qconj(Q));
    out = round(temp(2:4), 6);
    out(abs(out) < 1e-12) = 0;

    % %Winkel mit Korrektur
    % alpha = atan2((2*(Q(3)*Q(4)+Q(1)*Q(2))),(1-2*(Q(2)^2+Q(3)^2)));
    % beta = asin(2*(Q(1)*Q(3)-Q(2)*Q(4)));
    % gamma = atan2((2*(Q(2)*Q(3)+Q(1)*Q(4))),(1-2*(Q(3)^2+Q(4)^2)));

    %Winkel ohne Korrektur
    alpha = atan((2*(Q(3)*Q(4)+Q(1)*Q(2)))/(1-2*(Q(2)^2+Q(3)^2)));
    beta = asin(2*(Q(1)*Q(3)-Q(2)*Q(4)));
    gamma = atan((2*(Q(2)*Q(3)+Q(1)*Q(4)))/(1-2*(Q(3)^2+Q(4)^2)));

    tol = 1e-12;
    alpha(abs(alpha)<tol) = 0;
    beta(abs(beta)<tol)   = 0;
    gamma(abs(gamma)<tol) = 0;

    % Winkel in Grad
    alpha = rad2deg(alpha);
    beta  = rad2deg(beta);
    gamma = rad2deg(gamma);
end
%------------------------------------------------------------------------
x = [1, 2, -4, 5];
y = [-5,-1,3,2];
a = 5;

%a)
add = qadd(x, y);
norm = qnorm(x);
conj = qconj(x);
smult = qsmult(a, x);
normallize = qnormalize(x);

%Ausgabe
fprintf('Quaternion x: [%g %g %g %g]\n', x);
fprintf('Quaternion y: [%g %g %g %g]\n\n', y);
fprintf('Addition (x + y): [%g %g %g %g]\n', add);
fprintf('Norm von x: %g\n', norm);
fprintf('Konjugiertes von x: [%g %g %g %g]\n', conj);
fprintf('Skalarmultiplikation (%g * x): [%g %g %g %g]\n', a, smult);
fprintf('Normalisierte x: [%g %g %g %g]\n', normallize);
%------------------------------------------------------------------------
%b)
mult = qmult(x,y);
mult1 = qmult1(x,y);

%Ausgabe
fprintf('Multiplikation (x * y) nach Py Skript: [%g %g %g %g]\n', mult);
fprintf('Multiplikation (x * y) nach Matrix: [%g %g %g %g]\n', mult1);
%------------------------------------------------------------------------
%c)
v = [4,2,0];
u = [0,0,-1];
phi = -90;
rotate = qrotate(v, u, phi);

%Ausgabe
fprintf(['Ergebnis Drehung des Vektors v = [%g %g %g] um %g° ' ...
    'und dem Vektor u = [%g %g %g]: [%g %g %g]\n'],v, phi, u, rotate);
%------------------------------------------------------------------------
%d)
v = [1,0,0];
u = [2,2,0];
phi = 180;

[rotate2, Q, alpha, beta, gamma] = qrotate_plus(v, u, phi);

%Ausgabe
fprintf(['Ergebnis Drehung des Vektors v = [%g %g %g] um %g° ' ...
    'und dem Vektor u = [%g %g %g]: [%g %g %g]\n'],v, phi, u, rotate2);
fprintf(['Rotationsmatrix: [%g %g %g %g]\n Winkel' ...
    ' alpha: %g°\n Winkel beta: %g°\n Winkel gamma: %g°\n\n'], Q, alpha, beta, gamma);