function [xR1,xR2,xR3] =formasion( q, type, nor)

%Formation Control using APF method
%   This code is used to acheive formation b/w multiple agents using Firas
%   Artificial Potential Field funciton

close all
s=1;
k=1.5;
si=3;
n=7;
a=0.6;
ro=0.7;

% To be extern in test_structural.m
global xf1 yf1 xf3 yf3 xf2 yf2 xf4 yf4
global xp1 yp1 xp2 yp2 xp3 yp3 xp4 yp4 xR4

%xobs = [0;0];            % Obstacle position
dh=0.3;
dh1=1.6;
rad = 0.2;
xm = 0;
ym = 0;

% q=[x1(1);x1(2);x2(1);x2(2);x3(1);x3(2)];
x1 = q(1,:);
x2 = q(2,:);
x3 = q(3,:);
if nor==4
    x4 = q(4,:);
end

for i=1:nor
    xm = xm + q(i,1);
    ym = ym + q(i,2);
end
xm = xm/n;
ym = ym/n;

% xm = (x1(1)+x2(1)+x3(1))/3;  %mean x-axis
% ym = (x1(2)+x2(2)+x3(2))/3;  %mean x-axis

% Desired robot position as mean of their current position
% to attain trianglar formaiton
switch type
    case {'Triangle','triangle'}
        xR3=[xm,ym+1]; 
        xR2=[xm-2,ym-1];
        xR1=[xm+2,ym-1];
    case {'Line','Linear','line','linear'}
        xR1=[xm,ym]; 
        xR2=[xm-1,ym-1];
        xR3=[xm+1,ym+1]; 
    case {'Rectangle','rectangle'}
        xR1=[xm,ym+2]; 
        xR2=[xm-2,ym];
        xR3=[xm,ym-2]; 
        xR4=[xm+2,ym];
    otherwise
        disp('Error Invalid formation type entered');
end
dh=0.2;
rad = 0.3;

%drawing Target points to be attained
[xf1,yf1] = Draw_Object(dh, rad, xR1);
[xf2,yf2] = Draw_Object(dh, rad, xR2);
[xf3,yf3] = Draw_Object(dh, rad, xR3);
if nor==4
    [xf4,yf4] = Draw_Object(dh, rad, xR4);
end

%drawing Start robot position before Formation
[xo6,yo6] = Draw_Object(dh, rad, x1);
% rad2=0.1;
[xo7,yo7] = Draw_Object(dh, rad, x2);
[xo8,yo8] = Draw_Object(dh, rad, x3);
if nor==4
    [xo9,yo9] = Draw_Object(dh, rad, x4);
end

[X,Y] = meshgrid(-8:dh1:8);

%Va =0.5.*((X-xT(1)).^2+(Y-xT(2).^2));
% 1st Aproach
Vatt1 = Attr_Poten( X, Y, xR1 );
Vatt2 = Attr_Poten( X, Y, xR2 );
Vatt3 = Attr_Poten( X, Y, xR3 );
% Vatt1 =0.5.*((X-xR1(1)).^2+(Y-xR1(2)).^2);
% Vatt2 =0.5.*((X-xR2(1)).^2+(Y-xR2(2)).^2);
% Vatt3 =0.5.*((X-xR3(1)).^2+(Y-xR3(2)).^2);
if nor==4
%     Vatt4 =0.5.*((X-xR4(1)).^2+(Y-xR4(2)).^2);
    Vatt4 = Attr_Poten( X, Y, xR3 );
end

[fxa1,fya1] = gradient(Vatt1,dh1,dh1);
[fxa2,fya2] = gradient(Vatt2,dh1,dh1);
[fxa3,fya3] = gradient(Vatt3,dh1,dh1);
if nor==4 
    [fxa4,fya4] = gradient(Vatt4,dh1,dh1);
end

fX1 = -fxa1;
fY1 = -fya1;

fX2 = -fxa2;
fY2 = -fya2;

fX3 = -fxa3;
fY3 = -fya3;

if nor==4
    fX4 = -fxa4;
    fY4 = -fya4;
end
%figure
% quiver(X,Y,fX1+fX2+fX3+fX4,fY1+fY2+fY3+fY4,'b')
title('Total Potential funciton');
ylabel('APF(att) + APf(Rep)');
hold on

ss=1;
k=1;

xp1(1)=x1(1);
yp1(1)=x1(2);
xp2(1)=x2(1);
yp2(1)=x2(2);
xp3(1)=x3(1);
yp3(1)=x3(2);
if nor==4
    xp4(1)=x4(1);
    yp4(1)=x4(2);
end

ix1=[];
iy1=[];
jx1=[];
jy1=[];
fxx1=0;
fyy1=0;
h1=0;
ix2=[];
iy2=[];
jx2=[];
jy2=[];
fxx2=0;
fyy2=0;
h2=0;

ix3=[];
iy3=[];
jx3=[];
jy3=[];
fxx3=0;
fyy3=0;
h3=0;

if nor==4
ix4=[];
iy4=[];
jx4=[];
jy4=[];
fxx4=0;
fyy4=0;
h4=0;
end

fr =1;
while ss
Pw1=sqrt(((X-xp1(k)).^2)+((Y-yp1(k)).^2));

Pw2=sqrt(((X-xp2(k)).^2)+((Y-yp2(k)).^2));

Pw3=sqrt(((X-xp3(k)).^2)+((Y-yp3(k)).^2));

if nor==4
    Pw4=sqrt(((X-xp4(k)).^2)+((Y-yp4(k)).^2));
end
xw1(k)=min(min(Pw1));
[iix1,iiy1]=find(Pw1==xw1(k));
ix1(k)=iix1(1);
iy1(k)=iiy1(1);

xw2(k)=min(min(Pw2));
[iix2,iiy2]=find(Pw2==xw2(k));
ix2(k)=iix2(1);
iy2(k)=iiy2(1);

xw3(k)=min(min(Pw3));
[iix3,iiy3]=find(Pw3==xw3(k));
ix3(k)=iix3(1);
iy3(k)=iiy3(1);

if nor==4
xw4(k)=min(min(Pw4));
[iix4,iiy4]=find(Pw4==xw4(k));
ix4(k)=iix4(1);
iy4(k)=iiy4(1);
end

fx1=fX1(ix1(k),iy1(k));
fy1=fY1(ix1(k),iy1(k));
fxx1(k)=fx1./norm(fX1);
fyy1(k)=fy1./norm(fY1);
ff1(k,:)=[fX1(ix1(k),iy1(k)),fY1(ix1(k),iy1(k))];

% creating animation for Robot Formation
cla;
set_fplot(xo6,yo6,xo7,yo7,xo8,yo8,x1,x2,x3)

if nor==4
    
    fill(xo9,yo9,'y')
    txt4= 'Robot4';
    plot(xo9,yo9,'y')
    text(x4(1)-0.2,x4(2),txt4)
    plot(xp1(k),yp1(k),'or',xp2(k),yp2(k),'ob',xp3(k),yp3(k),'og',xp4(k),yp4(k),'oy','MarkerSize',7);

else
    plot(xp1(k),yp1(k),'or',xp2(k),yp2(k),'ob',xp3(k),yp3(k),'og','MarkerSize',7);
end
axis equal;
axis([-8,8,-8,8]);

drawnow;
M(fr) = getframe;
fr = fr+1;


xp1(k+1)=xp1(k)+dh1*(fxx1(k));
yp1(k+1)=yp1(k)+dh1*(fyy1(k));

fx2=fX2(ix2(k),iy2(k));
fy2=fY2(ix2(k),iy2(k));
fxx2(k)=fx2./norm(fX2);
fyy2(k)=fy2./norm(fY2);
ff2(k,:)=[fX2(ix2(k),iy2(k)),fY2(ix2(k),iy2(k))];
xp2(k+1)=xp2(k)+dh1*(fxx2(k));
yp2(k+1)=yp2(k)+dh1*(fyy2(k));


fx3=fX3(ix3(k),iy3(k));
fy3=fY3(ix3(k),iy3(k));
fxx3(k)=fx3./norm(fX3);
fyy3(k)=fy3./norm(fY3);
ff3(k,:)=[fX3(ix3(k),iy3(k)),fY3(ix3(k),iy3(k))];
xp3(k+1)=xp3(k)+dh1*(fxx3(k));
yp3(k+1)=yp3(k)+dh1*(fyy3(k));

if nor==4
fx4=fX4(ix4(k),iy4(k));
fy4=fY4(ix4(k),iy4(k));
fxx4(k)=fx4./norm(fX4);
fyy4(k)=fy4./norm(fY4);
ff4(k,:)=[fX4(ix4(k),iy4(k)),fY4(ix4(k),iy4(k))];
xp4(k+1)=xp4(k)+dh1*(fxx4(k));
yp4(k+1)=yp4(k)+dh1*(fyy4(k));
end
%xf3 = xf31+xp1(k+1);
%yf3 = yf31+yp1(k+1);
%show_robot(xf3,yf3,'b');

%xo8 = xo81+xp2(k+1);
%yo8 = yo81+yp2(k+1);
%show_robot(xo8,yo8,'r');
%plot(xp2(k+1),yp2(k+1),'r*');

if ((sqrt((xp1(k+1)-xR1(1)).^2+(yp1(k+1)-xR1(2)).^2)<=0.2)||(sqrt((xp2(k+1)-xR2(1)).^2+(yp2(k+1)-xR2(2)).^2)<=(0.2))||(sqrt((xp3(k+1)-xR3(1)).^2+(yp3(k+1)-xR3(2)).^2)<=0.2))
ss=0;
end
k=k+1;
end


% plot(xp1,yp1,'r--',xp2,yp2,'r--',xp3,yp3,'r--','MarkerSize',30);
%plot([xp],[yp],'r--*')
%numtimes=3;
%fps=10;
hold on
fill(xf1,yf1,'r')
txt1= 'Target1';
text(xR1(1)-0.2,xR1(2),txt1);
plot(xf1,yf1,'r')

fill(xf2,yf2,'r')
txt1= 'Target2';
text(xR2(1)-0.2,xR2(2),txt1);
plot(xf2,yf2,'r')

fill(xf3,yf3,'r')
txt1= 'Target3';
text(xR3(1)-0.2,xR3(2),txt1);
plot(xf3,yf3,'r')

if nor ==4
fill(xf4,yf4,'r')
txt1= 'Target3';
text(xR4(1)-0.2,xR4(2),txt1);
plot(xf4,yf4,'r')
end
   
if nor==4
line([xR1(1) xR2(1)],[xR1(2) xR2(2)])
line([xR4(1) xR3(1)],[xR4(2) xR3(2)])
line([xR2(1) xR3(1)],[xR2(2) xR3(2)])
line([xR4(1) xR1(1)],[xR4(2) xR1(2)])
else
    line([xR1(1) xR2(1)],[xR1(2) xR2(2)])
    line([xR1(1) xR3(1)],[xR1(2) xR3(2)])
    line([xR2(1) xR3(1)],[xR2(2) xR3(2)]) 
end
pause(2);


end

