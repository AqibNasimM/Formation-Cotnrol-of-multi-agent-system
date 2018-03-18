function set_fplot(xo2,yo2,xo3,yo3,xo4,yo4,x1,x2,x3)
title('Total Potential funciton');
ylabel('APF(att) + APf(Rep)');
hold on


fill(xo2,yo2,'b')
txt2= 'Robot1';
plot(xo2,yo2,'b')
text(x1(1)-0.2,x1(2),txt2)

fill(xo3,yo3,'g')
txt3= 'Robot2';
plot(xo3,yo3,'g')
text(x2(1)-0.2,x2(2),txt3)

fill(xo4,yo4,'y')
txt3= 'Robot3';
plot(xo4,yo4,'y')
text(x3(1)-0.2,x3(2),txt3)
end
