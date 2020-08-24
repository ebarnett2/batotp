function plotOutput

[isNewSpline,isNewPlots]=setEnvironment;

aSmoothWin=10; % moving average smoothing window for accel plots
lw=2; % line width for plotting

% read in configuration data
fid=fopen('../input/config.dat');
for i=1:5 fgetl(fid); end
nJoints=fscanf(fid,'%d',1); fgetl(fid);
nCart  =fscanf(fid,'%d',1); fgetl(fid);
for i=1:7; fgetl(fid); end
JntVelMax =fscanf(fid,'%f',nJoints)'; fgetl(fid); fgetl(fid);
JntAccMax =fscanf(fid,'%f',nJoints)'; fgetl(fid); 
isTrqConOn=fscanf(fid,'%d',1); fgetl(fid);
FrcMax =fscanf(fid,'%f',nJoints)'; fgetl(fid);
FrcMin =fscanf(fid,'%f',nJoints)'; fgetl(fid);
FrcMin(isnan(FrcMin))=-FrcMax(isnan(FrcMin));
isCartConOn=fscanf(fid,'%d',1); fgetl(fid);
CartVelMax=fscanf(fid,'%f',1); fgetl(fid); fgetl(fid);
CartAccMax=fscanf(fid,'%f',1); fgetl(fid);
fclose(fid);

% read in trajectory ydata
traj=trajRead('traj_out.dat',nJoints,nCart,aSmoothWin,isNewSpline);

% plot the output path in space
if(isCartConOn && nCart>2) spatialPlot(traj); end

ssdotPlot; % Phase plane plot

% plot joint positions
jointTrajPlot(traj,JntVelMax,JntAccMax,3,'Output joint trajectory',isNewPlots);

% plot cart. positions
if isCartConOn; cartTrajPlot(traj,CartVelMax,CartAccMax,4,'Output cart. trajectory',isNewPlots); end

% plot generalized forces
if isTrqConOn; genFrcPlot(traj,FrcMax,FrcMin,5,'Output joint torques',isNewPlots); end


%% plot the path in space
function spatialPlot(traj)
x=traj.cart(:,1);
y=traj.cart(:,2);
z=traj.cart(:,3);
figure(1); clf; plot3(x,y,z,'b-','LineWidth',.5); axis equal
xlabel('x'); ylabel('y'); zlabel('z'); 
set(gca,'plotboxaspectratio',[1 1 1])
  
%% Phase plane plot
function ssdotPlot
  
f1=figure(2); clf; hold on; set(f1,'Name','Phase plane plot')

fid=fopen('s-sdot.dat');
[s,sdot]=sdotRead(fid); plot(s,sdot,'-b','MarkerSize',1)
[s,sdot]=sdotRead(fid); plot(s,sdot,'-r','MarkerSize',1)
ymax=max(sdot); ymin=0; yrange=ymax-ymin;
ylims=ymin+[-.05 1.05]*yrange;
fclose(fid);

slims=[-.05 1.05]*s(end);
xlim(slims); ylim(ylims);

set(f1,'Position',[800    550   778   400]);
set(gca,'Position',[.08 .08 .9 .88])
xlabel('s'); ylabel('sdot');
L1=legend('after fwd. integ.','after rev. integ.');
set(L1,'Location','South');

%% read in traj. data
function traj=trajRead(fname,nJoints,nCart,aSmoothWin,isNewSpline)

traj={};
fid=fopen(fname);
tres=fread(fid,1,'float');
nPts=fread(fid,1,'int');
is_jntFull=fread(fid,1,'int');
traj.th=fread(fid,[nPts nJoints],'float');

traj.t=tres*(0:nPts-1);
th_pp=spline(traj.t,traj.th');

if isNewSpline
    thD_pp=fnder(th_pp);
    thDD_pp=fnder(thD_pp);

    traj.thD =fnval(thD_pp,traj.t)';
    traj.thDD=fnval(thDD_pp,traj.t)';
else
    thD_pp=ppder(th_pp);
    thDD_pp=ppder(thD_pp);

    traj.thD =ppval(thD_pp,traj.t)';
    traj.thDD=ppval(thDD_pp,traj.t)';
end

if aSmoothWin>0
  for i=1:nJoints; traj.thDD(:,i)=smoothe(traj.thDD(:,i),aSmoothWin); end
end
is_cartFull=fread(fid,1,'int');
if is_cartFull
    traj.cart=fread(fid,[nPts nCart],'float');
    cart_pp=spline(traj.t,traj.cart');

    if isNewSpline
        cartD_pp=fnder(cart_pp);
        cartDD_pp=fnder(cartD_pp);

        traj.cartD =fnval(cartD_pp,traj.t)';
        traj.cartDD=fnval(cartDD_pp,traj.t)';
    else
        cartD_pp=ppder(cart_pp);
        cartDD_pp=ppder(cartD_pp);

        traj.cartD =ppval(cartD_pp,traj.t)';
        traj.cartDD=ppval(cartDD_pp,traj.t)';
    end
    if aSmoothWin>0
       for i=1:nCart; traj.cartDD(:,i)=smoothe(traj.cartDD(:,i),aSmoothWin); end
    end
end
is_trqFull=fread(fid,1,'int');
if is_trqFull
    traj.genFrc=fread(fid,[nPts nJoints],'float'); 
    if aSmoothWin>0
      for i=1:nJoints; traj.genFrc(:,i)=smoothe(traj.genFrc(:,i),aSmoothWin); end
    end
end

fclose(fid);
traj.t=traj.t(:);

%% read in a phase-plane (s-sdot) curve
function [s,sdot]=sdotRead(fid)
tres=fread(fid,1,'double');
nPts=fread(fid,1,'int');
   s=fread(fid,nPts,'float');
sdot=fread(fid,nPts,'float');

%% plot the joint velocity and accel.
function jointTrajPlot(traj,JntVelMax,JntAccMax,fignum,title,isNewPlots)

nJoints=size(traj.th,2);
xL=[repmat(traj.t(1),1,nJoints); repmat(traj.t(end),1,nJoints)];
yLvelL=(-[1; 1]*JntVelMax);
yLvelH=( [1; 1]*JntVelMax);
yLaccL=(-[1; 1]*JntAccMax);
yLaccH=( [1; 1]*JntAccMax);

slims=[-.05 1.05]*traj.t(end);
f1=figure(fignum); clf; hold on; set(f1,'Name',title);
set(f1,'Position',[10    40   778   962]);

subplot(2,1,1); cla; hold on
h=plot(xL,yLvelL,':','Color',.7*[1 1 1]);
if isNewPlots;  resetPlotLineColors(h); end
   
h=plot(xL,yLvelH,':','Color',0.7*[1 1 1]);
if isNewPlots;  resetPlotLineColors(h); end

plot(traj.t,traj.thD);
ymax=max(traj.thD(:)); ymax=ymax+.001*abs(ymax)+1e-10;ymin=min(traj.thD(:)); yrange=ymax-ymin;
ylims=ymin+[-.05 1.05]*yrange;
xlim(slims);  ylim(ylims);
set(gca,'Position',[.08 .54 .9 .44])
ylabel('\theta-dot')

subplot(2,1,2); cla; hold on
h=plot(xL,yLaccL,':');
if isNewPlots;  resetPlotLineColors(h); end
h=plot(xL,yLaccH,':');
if isNewPlots;  resetPlotLineColors(h); end
plot(traj.t,traj.thDD);
ymax=max(traj.thDD(:)); ymax=ymax+.001*abs(ymax)+1e-10; ymin=min(traj.thDD(:)); yrange=ymax-ymin;
ylims=ymin+[-.05 1.05]*yrange;
xlim(slims);  ylim(ylims);
set(gca,'Position',[.08 .05 .9 .44])
xlabel('t [s]'); ylabel('\theta-ddot')

%% plot the Cartesian velocity and accel.
function cartTrajPlot(traj,CartVelMax,CartAccMax,fignum,title,isNewPlots)
f3=figure(fignum); clf; hold on; set(f3,'Name',title)
speed=sqrt(traj.cartD(:,1).^2 +traj.cartD(:,2).^2 +traj.cartD(:,3).^2);
accel=sqrt(traj.cartDD(:,1).^2+traj.cartDD(:,2).^2+traj.cartDD(:,3).^2);

set(f3,'Position',[10    40   778   962]);
slims=[-.05 1.05]*traj.t(end);
xL=[traj.t(1) traj.t(end)];

subplot(2,1,1); cla; hold on
plot(slims,[0 0],'k:')
plot(xL,CartVelMax*[1 1],'b:')
plot(traj.t,speed,'b');
ymax=max(speed); ymax=ymax+.001*abs(ymax)+1e-10; ymin=min(speed); yrange=ymax-ymin;
ylims=ymin+[-.05 1.05]*yrange;
xlim(slims);  ylim(ylims);
set(gca,'Position',[.08 .54 .9 .44])
ylabel('Cart. speed')

subplot(2,1,2); cla; hold on
plot(slims,[0 0],'k:')
plot(xL,CartAccMax*[1 1],'b:')
plot(traj.t,accel,'b');
ymax=max(accel); ymax=ymax+.001*abs(ymax)+1e-10;ymin=min(accel); yrange=ymax-ymin;
ylims=ymin+[-.05 1.05]*yrange;
xlim(slims);  ylim(ylims);
set(gca,'Position',[.08 .05 .9 .44])
xlabel('t [s]'); ylabel('Cart. accel.')

% plot the generalized forces
function genFrcPlot(traj,FrcMax,FrcMin,fignum,title,isNewPlots)

nJoints=size(traj.genFrc,2);
xL=[repmat(traj.t(1),1,nJoints); repmat(traj.t(end),1,nJoints)];
yLtrqL=([1; 1]*FrcMin);
yLtrqH=([1; 1]*FrcMax);

slims=[-.05 1.05]*traj.t(end);
f1=figure(fignum); clf; hold on; set(f1,'Name',title);
set(f1,'Position',[10    40   778   400]);

h=plot(xL,yLtrqL,':');
if isNewPlots;  resetPlotLineColors(h); end

h=plot(xL,yLtrqH,':');
if isNewPlots;  resetPlotLineColors(h); end

plot(traj.t,traj.genFrc);
ymax=max(traj.genFrc(:)); ymax=ymax+.001*abs(ymax)+1e-10; ymin=min(traj.genFrc(:)); yrange=ymax-ymin;
ylims=ymin+[-.05 1.05]*yrange;
xlim(slims);  ylim(ylims);
set(gca,'Position',[.08 .08 .9 .88])
ylabel('gen. force')

% 1-D data smoothing function
function x2=smoothe(x,w)

n=length(x);
n2=2*floor(n/2)-1;

w=min(w,n2);
x2=x;  if(n<5); return; end
w=2*floor(w/2)+1;
wMid=floor(w/2);
for i=1:wMid
    nPtsT=2*i+1;
    x2(i+1)=mean(x(1:nPtsT));
    x2(n-i)=mean(x(n-nPtsT+1:n));
end
xx=zeros(n-2*wMid,w);
for i=1:w
    xx(:,i)=x(i:i+n-w);
end
x2(wMid+1:n-wMid)=mean(xx,2);

function resetPlotLineColors(h)
set(gca, 'ColorOrder', circshift(get(gca, 'ColorOrder'), numel(h)))

%% set environment-dependent parameters
function [isNewSpline,isNewPlots]=setEnvironment

verStruct=ver('matlab');
isMATLAB=true;
if isempty(verStruct)
  isMATLAB=false; verStruct=ver('octave');
elseif isempty(verStruct.Name)
  isMATLAB=false; verStruct=ver('octave');
  graphics_toolkit('qt');
end

verA=str2double(verStruct.Version(1));

isNewSpline=false; isNewPlots=false;
if isMATLAB
    isNewSpline=true;
    if verA>7; isNewPlots=true; end
else
    isNewPlots=true;
    graphics_toolkit('qt') % other options are 'gnuplot' and 'fltk'
end
