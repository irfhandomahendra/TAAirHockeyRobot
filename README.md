# TAAirHockeyRobot
Air Hockey Robot (MATLAB + Arduino Mega)

blobdetection.m = code MATLAB untuk melakukan blob detecteion terhadap bola (puck) dan robot menggunakan PS3 Eye Camera.
TA_fix.ino = code arduino untuk menggerakkan robot. 
spesifikasi core robot : arduino mega 2560, ramps 1.4, driver motor stepper a4988
gerakan robot menggunakan 2 metode berbeda : 1. gerak robot ke arah samping (sumbu x) = Fuzzy Logic Controller
                                             2. gerak robot ke arah depan belakang (sumbu y) = PID > PD-I Controller
