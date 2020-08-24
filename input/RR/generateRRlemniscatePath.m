roundres=0.1;
theta=(180+(0:roundres:360))*pi/180;
theta=theta(end:-1:1);
xcent=.0; ycent=.6; xsca=.3; ysca=.2;
a1=.4; a2=.6;
xt=xsca*cos(theta); yt=ysca*sin(2*theta); z=zeros(size(xt));
y=xt+ycent; x=yt+xcent;
figure(1); plot(x,y); axis equal

[th1,th2] = IDP_RR(x,y,a1,a2,1);

fid=fopen('RRlemniscate.dat','w');
fwrite(fid,0.01,'float');
fwrite(fid,length(th1),'int');
fwrite(fid,1,'int'); % isThetaFull
fwrite(fid,180/pi*th1,'float');
fwrite(fid,180/pi*th2,'float');
fwrite(fid,0,'int'); % isCartFull
fclose(fid);
    
