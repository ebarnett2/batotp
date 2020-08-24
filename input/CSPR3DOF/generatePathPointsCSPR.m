% This MATLAB/Octave script generates a cubic spline interpolant through randomly
% selected points in the workspace of a 3DOF cable-suspended parallel mechanism
% The output format is a binary data file suitable for the BA algorithm

npts=20; s=0:npts-1; [x,y,z]=deal(zeros(npts,1));
amp=3;

for i=1:npts
    x(i)=amp*(rand(1,1)-.5);
    y(i)=amp*(rand(1,1)-.35);
    z(i)=amp*(rand(1,1)+.75);
end

xs=spline(s,x);
ys=spline(s,y);
zs=spline(s,z);

%sres=0.00005; % 330 000 points
sres=0.005
ss=0:sres:npts-1;
xx=ppval(xs,ss);
yy=ppval(ys,ss);
zz=ppval(zs,ss);

fid=fopen('CSPR3DOFspline.dat','w');
fwrite(fid,sres,'float');
fwrite(fid,length(xx),'int');
fwrite(fid,0,'int'); % path points do not contain joint positions
fwrite(fid,1,'int'); % path points do contain Cartesian positions
fwrite(fid,xx,'float');
fwrite(fid,yy,'float');
fwrite(fid,zz,'float');
fclose(fid);

xattach=[ 5.0655 -5.1958 0.1302  5.0655];
yattach=[-1.9978 -2.3085 4.3064 -1.9978];
zattach=[-0.0652 -0.0096 0.0747 -0.0652];

figure(10); clf; hold on; grid on; axis equal
decfact=sres/.005; decrange=1:decfact:length(ss);
plot3(xx(decrange),yy(decrange),zz(decrange));
plot3(x,y,z,'ro')

% The path must lie within the static workspace, which is the triangular prism
% formed by projecting this triangle downward (positive) in the z direction
plot3(xattach,yattach,zattach,'g')

