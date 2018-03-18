function [X,Y,fX,fY,xp,yp,k] = Track(x0 ,xT, a, n)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

s=1;
k=1.5;
si=3;
n=7;

% To be extern in test_structural.m
global Vatt Vobs xo1 yo1 xo2 yo2 xo3 yo3 xo5 yo5 xo6 yo6 xo7 yo7 xo8 yo8 xo9 yo9 xo10 yo10
global dh    %cell spacings

global xobs1 xobs2 xobs3 xobs4 xobs5 xobs6 xobs7     % Obstacle position
rad_obs = 0.8;
rad_rob = 0.6;
rad_tar = 0.3;

%drawing Obstacle point 1&2
[xo3,yo3] = Draw_Object(dh,rad_obs,xobs1);
[xo5,yo5] = Draw_Object(dh,rad_obs,xobs2);
if n>2
[xo6,yo6] = Draw_Object(dh,rad_obs,xobs3);
[xo7,yo7] = Draw_Object(dh,rad_obs,xobs4);
[xo8,yo8] = Draw_Object(dh,rad_obs,xobs5);
[xo9,yo9] = Draw_Object(dh,rad_obs,xobs6); 
[xo10,yo10] = Draw_Object(dh,rad_obs,xobs7);    
end

rad = 0.3;
% drawing Target point
[xo1,yo1] = Draw_Object(dh,rad_tar,xT);

%drawing Start robot position
[xo2,yo2] = Draw_Object(dh,rad_rob,x0);

[X,Y] = meshgrid(-8:dh:8);
%Va =0.5.*((X-xT(1)).^2+(Y-xT(2).^2));
% 1st Aproach
Vatt =((X-xT(1)).^2+(Y-xT(2)).^2);

Vobs1 = Rep_poten(X, Y, xobs1, rad_obs );
Vobs2 = Rep_poten(X, Y, xobs2, rad_obs );
if n>2
Vobs3 = Rep_poten(X, Y, xobs3, rad_obs );
Vobs4 = Rep_poten(X, Y, xobs4, rad_obs );
Vobs5 = Rep_poten(X, Y, xobs5, rad_obs );
Vobs6 = Rep_poten(X, Y, xobs6, rad_obs );
Vobs7 = Rep_poten(X, Y, xobs7, rad_obs );
end

if n>2
    Vobs = Vobs1+Vobs2+Vobs3+Vobs4+Vobs5+Vobs6+Vobs7;
else
    Vobs = Vobs1+Vobs2;
end
[fxa,fya] = gradient(Vatt,dh,dh);
[fxr,fyr] = gradient(Vobs,dh,dh);
%[DX,DY] = gradient(Va,dh,dh);
%  fX = -(1-a).*fxa-(a).*(fxr-fyr);
%  fY = -(1-a).*fya-(a).*(fyr+fxr);
  fX = -((1-a)*10).*fxa-(9*a).*(fxr-fyr);
  fY = -((1-a)*10).*fya-(9*a).*(fyr+fxr);

hold on
quiver(X,Y,fX,fY,'b')
title('Total Potential funciton');
ylabel('APF(att) + APf(Rep)');
hold on
fill(xo1,yo1,'r')
txt1= 'Target';
text(xT(1)-0.2,xT(2),txt1);
plot(xo1,yo1,'r')
fill(xo2,yo2,'b')
txt2= 'Start';
plot(xo2,yo2,'b')
text(x0(1)-0.2,x0(2),txt2)

fill(xo3,yo3,'b')
txt3= 'Obstacle';
plot(xo3,yo3,'b')
text(xobs1(1)-0.2,xobs1(2),txt3)

fill(xo5,yo5,'b')
txt3= 'Obstacle';
plot(xo5,yo5,'b')
text(xobs2(1)-0.2,xobs2(2),txt3)

ss=1;
k=1;
xp=[];
yp=[];
xp(1)=x0(1);
yp(1)=x0(2);
ix=[];
iy=[];
jx=[];
jy=[];
fxx=0;
fyy=0;
h=0;
while ss
Pw=sqrt(((X-xp(k)).^2)+((Y-yp(k)).^2));
xw(k)=min(min(Pw));
[iix,iiy]=find(Pw==xw(k));
ix(k)=iix(1);
iy(k)=iiy(1);
fx1=fX(ix(k),iy(k));
fy1=fY(ix(k),iy(k));
fxx(k)=fx1./norm(fX);
fyy(k)=fy1./norm(fY);
ff(k,:)=[fX(ix(k),iy(k)),fY(ix(k),iy(k))];
xp(k+1)=xp(k)+dh*(fxx(k));
yp(k+1)=yp(k)+dh*(fyy(k));
% hold on;
% plot(xp(k+1),yp(k+1),'--ro');
% pause(0.0001);
% plot(xp(k+1),yp(k+1),'--r');
% hold on
% axis equal;
% axis([-8,8,-8,8]);
% pause(0.00000000001);
if (sqrt((xp(k+1)-xT(1)).^2+(yp(k+1)-xT(2)).^2)<=(dh))
ss=0;
end
k=k+1;
end
hold
% plot([xp],[yp],'r--*')
% numtimes=3;
% fps=10;

end

