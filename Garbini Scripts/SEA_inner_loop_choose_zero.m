clear
close all


%---plant
wnp=sqrt(1.37e4);     %---plant natural frequency (rad/s)
Kp=1921;              %---plant gain       
p1=0+wnp*j;
p2=0-wnp*j;
plant=zpk([],[p1,p2],Kp);     %---plant
fprintf('plant natural frequency: %g Hz\n',wnp/2/pi);

%---phase lead compensation
Ts=.03;
zeta=.6;
wn=4/zeta/Ts;
locus=figure;
target=-zeta*wn+wn*sqrt(1-zeta^2)*j;

%---innerloop controller deisgn
zc=80;          %---controller zero 
v_zc=target+zc;
v_pato=target+p1; 
v_patm1=target+p2;
ang_pc=pi+angle(v_zc)-angle(v_pato)-angle(v_patm1);
pc=imag(target)/tan(ang_pc)-real(target);   %---controller pole
v_pc=target+pc;
Kc=abs(v_pato)*abs(v_patm1)*abs(v_pc)/abs(v_zc);   %---controller gain
lead=zpk([-zc],[-pc],Kc/Kp)                 %---lead compensator
gh=series(lead,plant);
subplot(211)
    rlocus(gh)  %---root locus
    a=axis; a(3)=-400; a(4)=400; axis(a); a=axis;
    axis equal

    hold on     %---draw design requirements
    plot(real(target)*[1,1],[-400,400],'r--')
    plot([0,-600*zeta],[0,600*sin(acos(zeta))],'r--')
    plot([0,-600*zeta],[0,-600*sin(acos(zeta))],'r--')

    r=rlocus(gh,1)  %---plot closed loop poles
    title('inner loop lead compensator')
    plot(r,'gs',    'MarkerEdgeColor','k',...
                    'MarkerFaceColor','g',...
                    'MarkerSize',9)
    axis equal

%---inner loop step response
subplot(212)
sys=feedback(gh,1);
sys
step(sys);
    title('inner loop lead compensator')
    
%---outer loop design (using PIDtune)
figure
Jout=1;
subplot(211)
    gh2 = series(sys,tf([1],[1/Jout 0]))
    wc=2*pi*8;      % control bandwidth (Gain crossover frequency)
    Options = pidtuneOptions('DesignFocus','reference-tracking');
    Cp = pidtune(gh2,'pidf',wc,Options)
    sys2=feedback(series(Cp,gh2),1)
    step(sys2)
    title('complete SAE response')
subplot(212)
    bode(sys2)
    title('Frequency Response - SAE')
%     
tilefigs
return            
     
