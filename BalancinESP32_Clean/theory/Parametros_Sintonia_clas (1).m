clc
clear

mM=0.097; % [Kg], masa de un motor
mC=0.009; % [kg], masa de un cople de rueda
mt=0.019; % [kg], masa de un tornillo (poste) con tuercas
mS=0.008; % [kg], masa de un soporte motor
mH=0.023; % [kg], masa tarjeta puente H
mB=0.147; % [kg], masa batería
mBo=0.040; % [kg], masa tarjeta microcontrolador
mA=0.026; % [kg], masa 1 hoja de acrílico
R=0.0694/2; % [m], radio rueda
mLL=0.062; %masa de llanta
mR=mLL+mC+mM/2; % [kg], Mw masa de una rueda

mTCalculada=2*mM+2*mLL+2*mC+4*mt+2*mS+mH+mB+mBo+2*mA

% Masa del péndulo sin considerar los motores
Mp=2*mS+mA+mB+mH+4*mt+mA+mBo

% Distancias importantes medidas desde eje de ruedas
x1=0.013;%distancia a primer acrílico
x2=0.0916;%distancia puente H
x3=0.072;%distancia a centro de masa de postes
x4=0.119;%distancia al segundo acrílico
x5=0.1285;%distancia a centro de masa de batería
x6=0.144;%distancia a la tarjeta
Cz=(2*mS*0.023/2 +mA*x1 +mB*x5 +mH*x2 +4*mt*x3 +mA*x4 +mBo*x6)/Mp

b=0.18/2;%mitad de la distancia entre huellas de llantas

Iyy_carcazas=(2*mM/2)*Cz^2;
Iyy_postes=4*mt*(abs(Cz-x3))^2/12;
Iyy_bateria=mB*(abs(Cz-x5))^2+mB*(0.034^2+0.025^2)/12;
Iyy_Bo=mBo*(0.050)^2/12+mBo*(abs(x6-Cz))^2;
Iyy_acrilico_inferior=mA*(0.05)^2/12+mA*(abs(Cz-x1))^2;
Iyy_acrilico_superior=mA*(0.05)^2/12+mA*(abs(x4-Cz))^2;
Iyy_H=mH*(abs(x2-Cz))^2;
Iyy=Iyy_carcazas+Iyy_postes+Iyy_bateria+Iyy_Bo+Iyy_acrilico_inferior+Iyy_acrilico_superior+Iyy_H

Iwa=mR*R^2/2;
Iwd=mR*(R^2/4+0.024^2/12);
g=9.81; 

M22=Mp*Cz^2+Iyy

Iphi=mR*R^2+Iwa;
sigma=Mp*Cz+2*Iphi/R;
sigma1=Mp*Cz*g;
sigma2=(1+Mp*Cz*R/(2*Iphi))/M22;

Gp=tf([sigma1*sigma2],[1 0 -sigma1/M22 0 0]);

figure(4)
rlocus(Gp)
grid on

z1=-2;
z2=-11.3;   
z3=-14.3;  

num1=conv([1 -z1],[1 -z2]);
nume=conv(num1,[1 -z3])
a0=nume(4)
a1=nume(3)
a2=nume(2)
a3=nume(1)

GH=Gp*tf(nume,1);
figure(2)
rlocus(GH)
axis([-120 20 -40 40])
%axis([-12 2 -4 4])
grid on
k=rlocfind(GH)
hold on

kdi=k*(Mp*Cz*g+a1*M22)
kpi=k*(a2*Mp*Cz*g+a0*M22)
kpv=k*a1*sigma
kiv=k*a0*sigma

grid on
hold on
