function [th1,th2] = IDP_RR(x,y,a1,a2,isRighty)
%% Solution of the inverse displacement problem for an RR planar manipulator
% Angeles, J. (2007), Fundamentals of Robotic Mechanical Systems, Third Edition, Springer, p. 193.

x=x(:); y=y(:); xsq=x.^2; ysq=y.^2;

c=a2^2-a1^2-xsq-ysq;
A=4*a1^2*(xsq+ysq); B=4*a1*x.*c; C=c.^2-4*a1^2*ysq;

c1=(-B+sqrt(B.^2-4*A.*C))./(2*A);
s1=(-c-2*a1*x.*c1)./(2*a1*y);
absy=abs(y); zeroy=absy<1e-6*max(absy);
s1(zeroy)=sqrt(1-c1(zeroy).^2);

th1=atan2(s1,c1); th2=real(atan2((y-a1*s1),(x-a1*c1)))-th1; ath2=abs(th2);
bigvals=ath2>pi; th2(bigvals)=sign(th2(bigvals)).*(ath2(bigvals)-2*pi);

if isRighty; switchVals=th2<0;
else         switchVals=th2>0; end
th2(switchVals)=-th2(switchVals);

thPswitch=atan2(y(switchVals),x(switchVals));
th1(switchVals)=2*thPswitch-th1(switchVals);
th1=shiftang(th1);

function ang=shiftang(ang)

fixranges=find(abs(diff(ang))>pi/2);
if(mod(length(fixranges),2)==1)
    fixranges=[fixranges; length(ang)];
end
for i=1:ceil(length(fixranges)/2)
    stpt=fixranges(2*i-1)+1; endpt=fixranges(2*i); reprange=stpt:endpt;
    ang(reprange)=ang(reprange)-2*pi*sign(ang(stpt));
end