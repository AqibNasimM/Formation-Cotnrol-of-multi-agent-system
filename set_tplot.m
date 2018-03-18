function set_tplot(X,Y,fX,fY,xT, n)

global xo1 yo1 xo3 yo3 xo5 yo5 xo6 yo6 xo7 yo7 xo8 yo8 xo9 yo9 xo10 yo10
global xobs1 xobs2 xobs3 xobs4 xobs5 xobs6 xobs7     % Obstacle position
hold on
quiver(X,Y,fX,fY,'b')
title('Total Potential funciton');
ylabel('APF(att) + APf(Rep)');
hold on
fill(xo1,yo1,'r')
txt1= 'Target';
text(xT(1)-0.2,xT(2),txt1);

fill(xo3,yo3,'b')
txt3= 'Obstacle';
plot(xo3,yo3,'b')
text(xobs1(1)-0.2,xobs1(2),txt3)

fill(xo5,yo5,'b')
txt3= 'Obstacle';
plot(xo5,yo5,'b')
text(xobs2(1)-0.2,xobs2(2),txt3)

if n>2
fill(xo6,yo6,'b')
txt3= 'Obstacle';
plot(xo6,yo6,'b')
text(xobs3(1)-0.2,xobs3(2),txt3)

fill(xo7,yo7,'b')
txt3= 'Obstacle';
plot(xo7,yo7,'b')
text(xobs4(1)-0.2,xobs4(2),txt3)

fill(xo8,yo8,'b')
txt3= 'Obstacle';
plot(xo8,yo8,'b')
text(xobs5(1)-0.2,xobs5(2),txt3)

fill(xo9,yo9,'b')
txt3= 'Obstacle';
plot(xo9,yo9,'b')
text(xobs6(1)-0.2,xobs6(2),txt3)

fill(xo10,yo10,'b')
txt3= 'Obstacle';
plot(xo10,yo10,'b')
text(xobs7(1)-0.2,xobs7(2),txt3)
end
end
