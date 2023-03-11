clear % clear workspace
close all % close all figures

Winkel = 330; % Winkel angeben
Radius = 0.8; % Radius angeben
Leistung = 1; % Leistung angeben (erstmal nicht gebraucht)

x0 = Radius * sin(Winkel * pi() / 180); % in x Koordinate umrechnen (die Daten bekommt man)
y0 = Radius * cos(Winkel * pi() / 180); % in y Koordinate umrechnen (die Daten bekommt man)
alpha_tilde = atan(abs(x0) ./ abs(y0)) / pi() * 180; % in Dummy Winkel umrechnen [deg]
r_i = sqrt(x0^2 + y0^2); % in Radius umrechnen

if x0 >= 0 && y0 >= 0 % Sektor 1

    alpha0 = alpha_tilde; % in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]

elseif x0 >= 0 && y0 < 0 % Sektor 2

    alpha0 = 180 - alpha_tilde; % in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]

elseif x0 < 0 && y0 < 0 % Sektor 3

    alpha0 = alpha_tilde + 180; % in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]

else % Sektor 4

    alpha0 = 360 - alpha_tilde; % in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]

end % if

if alpha0 >= 0 && alpha0 < 120 % Zwischen Motor A und B

    w_A = 0.5 + 0.5 * cos((1.5 * (alpha0 - 0)) * pi() / 180); % Anteil Motor A bei r = r_max
    w_B = 0.5 + 0.5 * cos((1.5 * (alpha0 - 120)) * pi() / 180); % Anteil Motor B bei r = r_max
    w_C = 1 - w_A - w_B; % Anteil Motor Cbei r = r_max

elseif alpha0 >= 120 && alpha0 < 240 % Zwischen Motor B und C

    w_B = 0.5 + 0.5 * cos((1.5 * (alpha0 - 120)) * pi() / 180); % Anteil Motor B bei r = r_max
    w_C = 0.5 + 0.5 * cos((1.5 * (alpha0 - 240)) * pi() / 180); % Anteil Motor C bei r = r_max
    w_A = 1 - w_B - w_C; % Anteil Motor A bei r = r_max

else  % Zwischen Motor C und A

    w_A = 0.5 + 0.5 * cos((1.5 * (alpha0 - 360)) * pi() / 180); % Anteil Motor A bei r = r_max
    w_C = 0.5 + 0.5 * cos((1.5 * (alpha0 - 240)) * pi() / 180); % Anteil Motor C bei r = r_max
    w_B = 1 - w_A - w_C; % Anteil Motor B bei r = r_max

end % if

w_A = (w_A - 1 / 3) * r_i + 1 / 3; % Anteil Motor A bei r = r_i
w_B = (w_B - 1 / 3) * r_i + 1 / 3; % Anteil Motor B bei r = r_i
w_C = (w_C - 1 / 3) * r_i + 1 / 3; % Anteil Motor C bei r = r_i