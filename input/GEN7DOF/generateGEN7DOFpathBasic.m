nPts=5;
path=5*rand(nPts,7);
fname=['GEN7DOFpathBasic.csv'];
fid=fopen(fname,'w');
fprintf(fid,' j1,  j2,  j3,  j4,  j5,  j6,  j7\n');
fprintf(fid,'%3.1f, %3.1f, %3.1f, %3.1f, %3.1f, %3.1f, %3.1f\n',path);
fclose(fid);
