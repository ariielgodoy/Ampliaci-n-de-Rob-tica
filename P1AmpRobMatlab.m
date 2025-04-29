Kw = 1;
R = 0.1;
Tau = 0.12;
K = 0.4;
Ts = 0.025;
t_max = 5;
N = round(t_max / Ts);

t = 0:Ts:t_max; % Vector de tiempo
%Condiciones iniciales
wi_ref = 1 * ones(size(t));  % Entrada escalón (mantener constante en el tiempo)
wd_ref = 3 * ones(size(t));  % Se puede cambiar para hacer giros
x = 0;
y = 0;
theta = 0;
X_hist = zeros(size(t));
Y_hist = zeros(size(t));
Theta_hist = zeros(size(t));


s = tf('s');
G_motor = 1/(0.12*s+1);

% Simular la respuesta del motor
wi = lsim(G_motor, wi_ref, t);
wd = lsim(G_motor, wd_ref, t);

for k = 1:length(t)
    v = (wd(k) + wi(k)) * R / 2;   % Velocidad lineal
    w = (wd(k) - wi(k)) * R / (2*K); % Velocidad angular
    
    % Integración numérica (método de Euler)
    x = x + v * cos(theta) * Ts;
    y = y + v * sin(theta) * Ts;
    theta = theta + w * Ts;
    
    % Guardar valores para graficar
    X_hist(k) = x;
    Y_hist(k) = y;
    Theta_hist(k) = theta;
end

% Graficar la trayectoria del vehículo
figure;
plot(X_hist, Y_hist, 'b', 'LineWidth', 2); hold on;
plot(X_hist(1), Y_hist(1), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Inicio
plot(X_hist(end), Y_hist(end), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Fin
xlabel('X (m)'); ylabel('Y (m)');
title('Trayectoria del Vehículo');
grid on; axis equal;
legend('Trayectoria', 'Inicio', 'Fin');