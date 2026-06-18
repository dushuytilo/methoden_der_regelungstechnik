clc
clear all
close all

[EV_3 EW_3] = eig([-8 32;-24 -12]) % Eigenwerte & Eigenvektoren für Kurzaufgabe 3

%Kurzaufgabe 4:
s = tf('s');
TF_s = 10/(3*s+1);

figure()
h = nyquistplot(TF_s);                                                      % Nyquist-Diagramm erstellen
setoptions(h, 'ShowFullContour', 'off')
title ('Frequenzgangsortskurve')                                            % nur untere Hälfte des Kreises
axis equal                                                                  % richtige Skalierung
theme light
exportgraphics(h, 'nyquist_kurzaufgabe_4.svg', 'ContentType', 'vector');    % Graph als .svg Datei speichern


% Projektaufgaben:
A = [0 1;0.4501 -0.6];
EW_projektaufgaben_A = eig(A)                                               %Eigenwerte zur Projektaufgabe, Kapitel 2.4