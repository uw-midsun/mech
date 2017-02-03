close all 
clear all

k = [0.0007,0.0015,0.004]
n = [0.5, .7, 1]
f = 0:0.1:100
hold on
plot(f,k(1)./(f.^n(1)),'displayName','Point')
plot(f,k(2)./(f.^n(2)),'displayName','Line')
plot(f,k(3)./(f.^n(3)),'displayName','Surface')
legend('show')
xlim([0,30])
ylim([0,0.005])
ylabel('Resistance (ohms)')
xlabel('Force (newtons)')