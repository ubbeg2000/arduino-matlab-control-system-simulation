clc; close all; clear;

% simulation settings
SAMPLING_FREQUENCY = 10; % gaboleh diganti
SIMULATION_DURATION = 10;
NUMBER_OF_SAMPLES = SIMULATION_DURATION * SAMPLING_FREQUENCY;

% motor model settings
MOTOR_GAIN = 28.375;
TIME_CONSTANT = 3.11; %0.05;

% controller settings
KP = 0.2;
KI = 0.01;
KD = 0.05;
MODE = "TEMP"; % bisa PERM ato TEMP

% variabel-variabel untuk memodelkan plant
cs = 0; % current speed
ci = 0; % current input
ps = 0; % previous speed
pi = 0; % previous input
sp = 0; % setpoint

% array untuk menyimpan hasil simulasi
sps = zeros(1,NUMBER_OF_SAMPLES);
css = zeros(1,NUMBER_OF_SAMPLES);
time = ((0:NUMBER_OF_SAMPLES-1)/SAMPLING_FREQUENCY);

% konfigurasi komunikasi serial
s = serialport('COM5', 115200);
s.flush();
s.configureTerminator("LF");

% penyetingan koefisien PID
s.readline();
s.writeline(MODE);
s.writeline(sprintf("%.2f", KP));
s.writeline(sprintf("%.2f", KI));
s.writeline(sprintf("%.2f", KD));

% precalculation buat model
c1 = MOTOR_GAIN / SAMPLING_FREQUENCY;
c2 = (2 * TIME_CONSTANT) - 1/SAMPLING_FREQUENCY;
c3 = (2 * TIME_CONSTANT) + 1/SAMPLING_FREQUENCY;

% penyetingan monitor
figure(1);
p=plot(time,css, (0:NUMBER_OF_SAMPLES-1)/SAMPLING_FREQUENCY, sps);
title("Setpoint and Response");
xlabel("Time (s)");
ylabel("Motor Speed (rad/s)");
legend(["Response", "Setpoint"])
grid on;

tic();

% simulasi
i = 1;
while(i <= NUMBER_OF_SAMPLES)
    while s.NumBytesAvailable == 0
    end

    pi = ci;
    ci = str2double(s.readline());
    sp = str2double(s.readline());

    ps = cs;
    cs = (c1 * (ci +  pi) + c2 * ps)/c3;

    sps(i) = sp;
    css(i) = cs;

    s.writeline(sprintf("%.2f", cs));

    p(1).set('YData', css);
    p(2).set('YData', sps);
    drawnow;

    i = i + 1;
end

toc()

s.flush();
s.delete();

f = fopen("sim.csv", "w");
fprintf(f, "%s,%s,%s\n", "time", "setpoint", "speed");
fprintf(f, "%.2f,%.2f,%.2f\n", [time; sps; css]);
fclose(f);
