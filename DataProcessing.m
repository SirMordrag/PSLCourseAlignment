%% INIT
clc, clear all, close all;

load('SFin.mat'); SFin = SFin';
load('Win.mat'); Win = Win';

nsamples_G = size(Win,1);
nsamples_A = size(SFin,1);
assert(nsamples_A == nsamples_G);
nsamples = nsamples_G;

%% RAW ACC AND GYRO DATA
figure
hold on

subplot(3,2,1)
plot(1:nsamples, Win(1:nsamples,1))

subplot(3,2,3)
plot(1:nsamples, Win(1:nsamples,2))

subplot(3,2,5)
plot(1:nsamples, Win(1:nsamples,3))

subplot(3,2,2)
plot(1:nsamples, SFin(1:nsamples,1))

subplot(3,2,4)
plot(1:nsamples, SFin(1:nsamples,2))

subplot(3,2,6)
plot(1:nsamples, SFin(1:nsamples,3))

%% single-side FFT with freq. amplitude adjustment
figure
hold on

Fs = 2000;

subplot(3,2,1)
[f_accx,f] = my_fft(SFin(:,1), Fs);
plot(f, f_accx)

subplot(3,2,3)
[f_accy,f] = my_fft(SFin(:,2), Fs);
plot(f, f_accy)

subplot(3,2,5)
[f_accz,f] = my_fft(SFin(:,3), Fs);
plot(f, f_accz)

subplot(3,2,2)
[f_gyrox,f] = my_fft(Win(:,1), Fs);
plot(f, f_gyrox)

subplot(3,2,4)
[f_gyroy,f] = my_fft(Win(:,2), Fs);
plot(f, f_gyroy)

subplot(3,2,6)
[f_gyroz,f] = my_fft(Win(:,3), Fs);
plot(f, f_gyroz)

%% ACC raw vs lowpass 100 Hz
figure
hold on

Fs = 2000;

subplot(3,2,1)
[f_accx,f] = my_fft(SFin(:,1), Fs);
plot(f, f_accx)

subplot(3,2,3)
[f_accy,f] = my_fft(SFin(:,2), Fs);
plot(f, f_accy)

subplot(3,2,5)
[f_accz,f] = my_fft(SFin(:,3), Fs);
plot(f, f_accz)

subplot(3,2,2)
[f_accx,f] = my_fft(lowpass(SFin(:,1),100,Fs), Fs);
plot(f, f_accx)
grid minor
legend(["accx - lowpass"])
xlim([0 300])

subplot(3,2,4)
[f_accy,f] = my_fft(lowpass(SFin(:,2),100,Fs), Fs);
plot(f, f_accy)
grid minor
legend(["accy - lowpass"])
xlim([0 300])

subplot(3,2,6)
[f_accz,f] = my_fft(lowpass(SFin(:,3),100,Fs), Fs);
plot(f, f_accz)
grid minor
legend(["accz - lowpass"])
xlim([0 300])

%% GYRO raw vs lowpass 100 Hz
figure
hold on

Fs = 2000;

subplot(3,2,1)
[f_gyrox,f] = my_fft(Win(:,1), Fs);
plot(f, f_gyrox)

subplot(3,2,3)
[f_gyroy,f] = my_fft(Win(:,2), Fs);
plot(f, f_gyroy)

subplot(3,2,5)
[f_gyroz,f] = my_fft(Win(:,3), Fs);
plot(f, f_gyroz)

subplot(3,2,2)
[f_gyrox,f] = my_fft(lowpass(Win(:,1),100,Fs), Fs);
plot(f, f_gyrox)
grid minor
legend(["gx - lowpass"])
xlim([0 300])

subplot(3,2,4)
[f_gyroy,f] = my_fft(lowpass(Win(:,2),100,Fs), Fs);
plot(f, f_gyroy)
grid minor
legend(["gy - lowpass"])
xlim([0 300])

subplot(3,2,6)
[f_gyroz,f] = my_fft(lowpass(Win(:,3),100,Fs), Fs);
plot(f, f_gyroz)
grid minor
legend(["gz - lowpass"])
xlim([0 300])

%% AVAR

Fs = 2000;

runAVAR(Win, 1, Fs, 'Gyro X')
runAVAR(Win, 2, Fs, 'Gyro Y')
runAVAR(Win, 3, Fs, 'Gyro Z')
 
runAVAR(SFin, 1, Fs, 'Acc X')
runAVAR(SFin, 2, Fs, 'Acc Y')
runAVAR(SFin, 3, Fs, 'Acc Z')