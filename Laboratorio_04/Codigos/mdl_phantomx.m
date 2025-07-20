% MDL_PHANTOMX - Modelo PhantomX Pincher 
clear; close all; clc;

% Conversión de grados a radianes
deg = pi/180;

% Definición de eslabones usando parámetros DH estándar y dimensiones reales
L(1) = Link('d', 0.069,  'a', 0,      'alpha', pi/2,  'qlim', [-180 180]*deg);  % Base -> Hombro
L(2) = Link('d', 0,      'a', 0.103,  'alpha', 0,     'qlim', [0 180]*deg);    % Hombro -> Codo
L(3) = Link('d', 0,      'a', 0.1033,  'alpha', 0,     'qlim', [-100 100]*deg);  % Codo -> Muñeca
L(4) = Link('d', 0,      'a', 0.091175,  'alpha', 0,     'qlim', [-160 160]*deg);  % Muñeca (sin pinza)

% Crear robot como SerialLink
phantomx = SerialLink(L, 'name', 'PhantomX Pincher');

% Ajustar la base para levantar el robot del plano XY
phantomx.base = transl(0, 0, 0.05);

% Confirmación
disp(' Modelo PhantomX Pincher creado correctamente en la variable: phantomx');

% Configuración articular inicial (todo en cero)
q0 = zeros(1, phantomx.n);

% Graficar robot con escala y espacio de trabajo amplio
figure('Name','PhantomX Pincher - Escala Real');
phantomx.plot(q0, ...
    'workspace', [-0.3 0.3 -0.3 0.3 0 0.6], ...  % Espacio amplio para moverse
    'scale', 1.0, ...                           % Escala 1:1 (real)
    'view', [135 25], ...                       % Vista isométrica
    'tilesize', 0.05, ...
    'floorlevel', 0);

title('PhantomX Pincher');

% Panel de control interactivo
phantomx.teach
