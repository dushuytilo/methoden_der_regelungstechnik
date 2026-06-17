m = 2.5;      % Masse
l = 12.5;     % Länge
d = 1.5;      % Dämpfungskonstante
g = 9.81;     % Fallbeschleunigung

phi_r = deg2rad(125);     % Ruhelage in Rad
phi_0 = deg2rad(25);      % Anfangsauslenkung relativ zu phi_r in Rad
x_d_0 = 0;                % Anfangswinkelgeschwindigkeit phi_dot

M_A_r = l*m*g*sin(phi_r); % Moment für vorgegebene Ruhelage

D = 0.45;                 % gegebener Dämpfungsgrad
k_v = (l*d)^2/(4*m*D^2) - g*m*l*cos(phi_r); %Verstärkungsfaktor des Reglers

f = 50;             % Frequenz in Hz
dt = 1/f;           % Euler-Schritt in s
t_end = 20;         % Simulationszeit in s
t = 0:dt:t_end;     % Vektor der Zeitstamps, Länge t_end
N = length(t);

%Initialisierung, Matrizen erstellen
x_ungeregelt = zeros(2,N);          % nichtlinear ungeregelt
x_geregelt = zeros(2,N);            % nichtlinear geregelt

delta_x_ungeregelt = zeros(2,N);    % linear ungeregelt
delta_x_geregelt = zeros(2,N);      % linear geregelt

% Für nicht lineares System --- absolute Auslenkung vom Nullpunkt
x_ungeregelt(:,1) = [phi_r + phi_0; x_d_0];
x_geregelt(:,1)   = [phi_r + phi_0; x_d_0];

% Für linearisiertes System --- Abweichung vom Arbeitspunkt!!!
delta_x_ungeregelt(:,1) = [phi_0; x_d_0];
delta_x_geregelt(:,1)   = [phi_0; x_d_0];

%% Euler-Verfahren

for i = 2:N

    % Nichtlinear ungeregelt
    x_ungeregelt(:,i) = x_ungeregelt(:,i-1) ...
        + dt * Pendel(x_ungeregelt(:,i-1), m, l, d, g, M_A_r);

    % Nichtlinear geregelt
    x_geregelt(:,i) = x_geregelt(:,i-1) ...
        + dt * Pendel_geregelt(x_geregelt(:,i-1), m, l, d, g, phi_r, M_A_r, k_v);

    % Linearisiert ungeregelt
    delta_x_ungeregelt(:,i) = delta_x_ungeregelt(:,i-1) ...
        + dt * Pendel_lin(delta_x_ungeregelt(:,i-1), m, l, d, g, phi_r);

    % Linearisiert geregelt
    delta_x_geregelt(:,i) = delta_x_geregelt(:,i-1) ...
        + dt * Pendel_lin_geregelt(delta_x_geregelt(:,i-1), m, l, d, g, phi_r, k_v);

end

%% Zustände

% Lineare Zustände sind Abweichungen vom Arbeitspunkt
% Für den Vergleich mit dem nichtlinearen System werden sie zurückgerechnet:
phi_lin_ungeregelt = phi_r + delta_x_ungeregelt(1,:);
phi_lin_geregelt   = phi_r + delta_x_geregelt(1,:);

phi_dot_lin_ungeregelt = delta_x_ungeregelt(2,:);
phi_dot_lin_geregelt   = delta_x_geregelt(2,:);


%% Stellgröße/n
% Nichtlinear 
% Motormoment M_A

M_A_ungeregelt = M_A_r * ones(1,N);
M_A_geregelt   = M_A_r - k_v * (x_geregelt(1,:) - phi_r);

% Linear
% M_A = M_A_r + delta_u
delta_u_ungeregelt = zeros(1,N);
delta_u_geregelt   = -k_v * delta_x_geregelt(1,:);

M_A_lin_ungeregelt = M_A_r + delta_u_ungeregelt;
M_A_lin_geregelt   = M_A_r + delta_u_geregelt;

%% Plots.
% Keine Regelung
figure;

subplot(3,1,1);
plot(t, rad2deg(x_ungeregelt(1,:)), 'LineWidth', 1.5);
hold on;
plot(t, rad2deg(phi_lin_ungeregelt), '--', 'LineWidth', 1.5);
grid on;
xlabel('t [s]');
ylabel('\phi [deg]');
legend('nichtlinear', 'linearisiert');
title('Ungeregelt: Auslenkung');

subplot(3,1,2);
plot(t, rad2deg(x_ungeregelt(2,:)), 'LineWidth', 1.5);
hold on;
plot(t, rad2deg(phi_dot_lin_ungeregelt), '--', 'LineWidth', 1.5);
grid on;
xlabel('t [s]');
ylabel('d\phi/dt [deg/s]');
legend('nichtlinear', 'linearisiert');
title('Ungeregelt: Winkelgeschwindigkeit');

subplot(3,1,3);
plot(t, M_A_ungeregelt, 'LineWidth', 1.5);
hold on;
plot(t, M_A_lin_ungeregelt, '--', 'LineWidth', 1.5);
grid on;
xlabel('t [s]');
ylabel('M_A [Nm]');
legend('nichtlinear', 'linearisiert');
title('Ungeregelt: Stellgröße');

%geregelt

figure;

subplot(3,1,1);
plot(t, rad2deg(x_geregelt(1,:)), 'LineWidth', 1.5);
hold on;
plot(t, rad2deg(phi_lin_geregelt), '--', 'LineWidth', 1.5);
yline(rad2deg(phi_r), ':', 'Sollwinkel');
grid on;
xlabel('t [s]');
ylabel('\phi [deg]');
legend('nichtlinear', 'linearisiert', 'Sollwinkel');
title('Geregelt: Auslenkung');

subplot(3,1,2);
plot(t, rad2deg(x_geregelt(2,:)), 'LineWidth', 1.5);
hold on;
plot(t, rad2deg(phi_dot_lin_geregelt), '--', 'LineWidth', 1.5);
grid on;
xlabel('t [s]');
ylabel('d\phi/dt [deg/s]');
legend('nichtlinear', 'linearisiert');
title('Geregelt: Winkelgeschwindigkeit');

subplot(3,1,3);
plot(t, M_A_geregelt, 'LineWidth', 1.5);
hold on;
plot(t, M_A_lin_geregelt, '--', 'LineWidth', 1.5);
yline(M_A_r, ':', 'Haltemoment');
grid on;
xlabel('t [s]');
ylabel('M_A [Nm]');
legend('nichtlinear', 'linearisiert', 'Haltemoment');
title('Geregelt: Stellgröße');


%% Modelle

function F = Pendel(x, m, l, d, g, M_A_r)

    phi = x(1); 
    phi_dot = x(2);

    M_A = M_A_r;  

    F = zeros(2,1);

    F(1) = phi_dot;
    F(2) = (1/(m*l^2))*M_A ...
           - (d/m)*phi_dot ...
           - (g/l)*sin(phi);

end

function F = Pendel_geregelt(x, m, l, d, g, phi_r, M_A_r, k_v)

    phi = x(1);
    phi_dot = x(2);

    % Regeldifferenz bezogen auf phi_r
    delta_u = -k_v*(phi - phi_r);

    % gesamtes Motormoment
    M_A = M_A_r + delta_u;

    F = zeros(2,1);

    F(1) = phi_dot;
    F(2) = (1/(m*l^2))*M_A ...
           - (d/m)*phi_dot ...
           - (g/l)*sin(phi);

end

function F = Pendel_lin(delta_x, m, l, d, g, phi_r)

    A = [0, 1;
        -(g/l)*cos(phi_r), -d/m];

    B = [0;
         1/(m*l^2)];

    delta_u = 0;

    F = A*delta_x + B*delta_u;

end

function F = Pendel_lin_geregelt(delta_x, m, l, d, g, phi_r, k_v)

    A = [0, 1;
        -(g/l)*cos(phi_r), -d/m];

    B = [0;
         1/(m*l^2)];

    C = [1, 0];

    % delta_u = -k_v * delta_phi
    delta_u = -k_v*C*delta_x;

    F = A*delta_x + B*delta_u;

end


