ROpos = Device.qPOS('x y z');
steps = linspace(-4,4,91);
xdata = nan(length(steps),1);
ddata = nan(length(steps),1);
for i = 1:length(steps)
    Device.MOV('x',ROpos(1)+steps(i));
    Device.WAIT()
    Device.DRC(1,'x',2);
    Device.DRC(4,'1',17);
    Device.DRT ( 2, 1, '0' );
    Device.DRT ( 2, 4, '0' );
    pause(1);
    RegX = Device.qDRR(1,1,-1);
    tlen = length(RegX(:,1));
    RegD = Device.qDRR(4,1,tlen);
    xdata(i) = mean(RegX(end-20:end,2));
    ddata(i) = mean(RegD(end-20:end,2));
end
xdata = xdata - ROpos(1);

