fid=fopen('trajKuka.dat');
DecFact=fread(fid,1,'double');
nPts=fread(fid,1,'double');
fread(fid,nPts,'single'); % sdotbnd, not used in recorded traj
theta=fread(fid,[nPts 7],'single');
fclose(fid);

plot(theta)

fid=fopen('KUKApath.dat','w');
fwrite(fid,DecFact*0.01,'float');
fwrite(fid,nPts,'int');
fwrite(fid,1,'int'); % isThetaFull
fwrite(fid,theta,'single');
fwrite(fid,0,'int'); % isCartFull
fclose(fid);
