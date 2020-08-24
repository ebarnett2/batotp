nIter=1;
nPts=20*ones(nIter,1);
for iter=1:nIter
    nPtsA=nPts(iter);
    nPtsB=20*nPtsA;
    
    path=5*rand(nPtsA,7);
    s_in=0:nPtsA-1;
    s_out=linspace(0,nPtsA-1,nPtsB);
    path=interp1(s_in,path,s_out,'spline')';
    
    fname=['GEN7DOFpath',num2str(iter,'%04d'),'.dat'];
    fid=fopen(fname,'w');
    fwrite(fid,0.01,'float');
    fwrite(fid,nPtsB,'int');
    fwrite(fid,1,'int'); % isThetaFull
    fwrite(fid,path','float');
    fwrite(fid,0,'int'); % isCartFull
    fclose(fid);
    
    fname=['GEN7DOFpath',num2str(iter,'%04d'),'.csv'];
    fid=fopen(fname,'w');
    timestamp=0.01*(0:nPtsB-1);
    path=[timestamp; path];
    fprintf(fid,'timestamp,j1,j2,j3,j4,j5,j6,j7\n');
    
    fprintf(fid,'%f,%f,%f,%f,%f,%f,%f,%f\n',path);
    fclose(fid);
    
end
