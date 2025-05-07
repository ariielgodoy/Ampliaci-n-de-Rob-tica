clear all
close all
clc

% Entorno cerrado definido por una lista de puntos
x=[0 30 30 0 0]';
y=[0 0 10 10 0]';


%Posiciones iniciales del robot
x0 = 4;
y0 = 5;
phi0 = pi;


rangos = laser2D(x, y, x0, y0, phi0);


%Descripcion del robot
R = 0.1;
v = 0.3;
K = 0.4;
tau = 0.12;
s = tf('s');
T = 0.1;
G_motor = 1/(tau * s + 1); 
Gd_motor = c2d(G_motor, T);
[A, B, C, D] = ssdata(Gd_motor);

%Condicion inicial de los motores
x_motor_d = 0;
x_motor_i = 0;


tiempo_odometria = 0:T:90




% SACAR POR PANTALLA EL INICIO
figure;
hold on;
D_total = dibujaBarrido(x, y, x0, y0, phi0, rangos)
grid on;
plot(x0, y0, '*'); % Posición del robot inicial
grid on;
axis equal;
Xanterior = 0;
tbarridolidar = 0;
for tiempo = 0:T:90

    if x0 > 29
        break;
    end
    dL = 5 - y0;
    diferencia_angulo = wrapToPi(acos(10 / D_total));

    if dL < 0
        diferencia_angulo = -diferencia_angulo;
    end
    d = 1;
    Xobjetivo = dL*sin(diferencia_angulo + phi0) + (d)*cos(diferencia_angulo+phi0);
    Xobjetivo = real(Xobjetivo);
    Yobjetivo = dL*cos(diferencia_angulo+phi0) - (d)*sin(diferencia_angulo+phi0);
    Yobjetivo = real(Yobjetivo);
    dObjetivo = sqrt(Xobjetivo^2 + Yobjetivo^2);
    curvatura = 2*Yobjetivo/dObjetivo^2;

    alpha = wrapToPi(atan2(real(Yobjetivo), real(Xobjetivo)));
    plot(Xobjetivo + x0, Yobjetivo + y0, 'ro', 'MarkerSize', 2, 'MarkerFaceColor', 'g');
    w_ref = v * curvatura;

    A_ruedas = [1 1; 1 -1];
    B_ruedas = [v*2/R; w_ref*2*K/R];
    ruedas = A_ruedas \ B_ruedas;
    wd_ideal = ruedas(1);
    wi_ideal = ruedas(2);
    wd_real = C * x_motor_d + D * wd_ideal;
    x_motor_d = A * x_motor_d + B * wd_ideal;

    wi_real = C * x_motor_i + D * wi_ideal;
    x_motor_i = A * x_motor_i + B * wi_ideal;
    w_real = (wd_real - wi_real) * R / (2*K);


    phi0 = wrapToPi(phi0 + w_real * T);
    x0 = x0 + real(v * cos(phi0) *T);
    y0 = y0 + real(v * sin(phi0) *T);
    if tiempo - tbarridolidar >= 0.5
        rangos = laser2D(x, y, x0, y0, phi0);
        D_total = dibujaBarrido(x, y, x0, y0, phi0, rangos);
        tbarridolidar = tiempo;
    end


    %admito haber usado IA para la representacion
    longitud_flecha = 0.4;
    ancho_flecha = 0.2;
    p1x = x0 + longitud_flecha * cos(phi0);
    p1y = y0 + longitud_flecha * sin(phi0);
    p2x = x0 + ancho_flecha * cos(phi0+ alpha + 2*pi/3); % Vértice a la izquierda
    p2y = y0 + ancho_flecha * sin(phi0+ alpha + 2*pi/3);
    p3x = x0 + ancho_flecha * cos(phi0+ alpha - 2*pi/3); % Vértice a la derecha
    p3y = y0 + ancho_flecha * sin(phi0+ alpha - 2*pi/3);

    % Dibuja el triángulo
    plot([p1x p2x p3x p1x], [p1y p2y p3y p1y], 'b-', 'LineWidth', 1);
    patch([p1x p2x p3x], [p1y p2y p3y], 'r');
    drawnow
end


