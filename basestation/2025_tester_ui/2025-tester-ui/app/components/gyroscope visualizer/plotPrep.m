i = 0;

figure1=figure('Color',[0.30,0.30,0.30],'InvertHardcopy','off');

ax1 = subplot(3,3,[1,2,3]);
set(gca,'color',[0.24,0.24,0.24], 'XColor',[1 1 1], 'YColor',[1 1 1]);
% ax1.Position = [0 0.5 0.5 0.5];
%ax1.title = "Acclearation and Gyroscope";
xlabel(ax1,'time')
ylabel(ax1,'Acclerometer and gyroscope')

hold on;
line1 = line(i, rand(1));  
line2 = line(i, rand(1));  
line3 = line(i, rand(1));  
line4 = line(i, rand(1));
line5 = line(i, rand(1));  
line6 = line(i, rand(1));

i = i+1;

ax2 = subplot(3,3,8);
set(gca,'color',[0.24,0.24,0.24], 'XColor',[1 1 1], 'YColor',[1 1 1]);
% ax2.Position = [0.53 0.5 0.48 0.5];
% xlabel(ax2,'time')
% ylabel(ax2,'Acclerometer')

line7 = line(i, rand(1));  
line8 = line(i, rand(1));  
line9 = line(i, rand(1)); 

ax3 = subplot(3,3,7);
set(gca,'color',[0.24,0.24,0.24], 'XColor',[1 1 1], 'YColor',[1 1 1]);
% ax3.Position = [0 0 0.5 0.5];
xlabel(ax3,'time')
ylabel(ax3,'Acclerometer')

line10 = line(i, rand(1));
line11 = line(i, rand(1));  
line12 = line(i, rand(1));

ax4 = subplot(3,3,4);
set(gca,'color',[0.24,0.24,0.24],'xticklabel',[], 'XColor',[1 1 1], 'YColor',[1 1 1]);
% ax4.Position = [0.53 0 0.46 0.48];
xlabel(ax4,'time')
ylabel(ax4,'Gyroscope')
% ax4.YLabel = "Accleration and gyro";
% ax4.XLabel = "Time";

line13 = line(i, rand(1));
line14 = line(i, rand(1));  
line15 = line(i, rand(1));

ax5 = subplot(3,3,5);
set(gca,'color',[0.24,0.24,0.24], 'XColor',[1 1 1], 'YColor',[1 1 1]);
xlabel(ax5,'time')
ylabel(ax5,'Gyroscope Svatisky Golay (20%)')

line16 = line(i, rand(1));
line17 = line(i, rand(1));  
line18 = line(i, rand(1));

ax6 = subplot(3,3,6);
set(gca,'color',[0.24,0.24,0.24], 'XColor',[1 1 1], 'YColor',[1 1 1]);
xlabel(ax6,'time')
ylabel(ax6,'Gyro Chebyshev filter Low pass')

line19 = line(i, rand(1));
line20 = line(i, rand(1));  
line21 = line(i, rand(1));

xc=0; yc=0; zc=0;    % coordinated of the center
L=2;                 % cube size (length of an edge)
alpha=0.8;             % transparency (max=1=opaque)

X = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
Y = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
Z = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];

C= [0.1 0.5 0.9 0.9 0.1 0.5];   % color/face

X = L*(X-0.5) + xc;
Y = L/1.5*(Y-0.5) + yc;
Z = L/3*(Z-0.5) + zc;
V=[reshape(X,1,24); reshape(Y,1,24); reshape(Z,1,24)]; %rashape takesthe element of X and it fix them in only one coulomn (in this case)
ax7 = subplot(3,3,9);
set(gca,'color',[0.24,0.24,0.24], 'XColor',[1 1 1], 'YColor',[1 1 1]);

% fill3(X,Y,Z,C)
% axis square

x_l = [];
x_r13 = [];
x_r14 = [];
x_r15 = [];

[x_r16,x_r17 ,x_r18] = deal([]);

while(1)

    pause(0.10); 
    
    line1.XData = [line1.XData i];
    line1.YData = [line1.YData rand(1)];
    line1.Color = [1 0 0];
    line1.LineWidth = 1;
    line1.LineStyle = '-';
   
    line2.XData = [line2.XData i];
    line2.YData = [line2.YData rand(1)];
    line2.Color = [1 1 0];
    line2.LineWidth = 1;
    line2.LineStyle = '-';
    
    line3.XData = [line3.XData i];
    line3.YData = [line3.YData rand(1)];
    line3.Color = [0 1 0];
    line3.LineWidth = 1;
    line3.LineStyle = '-';

     line4.XData = [line4.XData i];
    line4.YData = [line4.YData rand(1)];
    line4.Color = [0 1 0];
    line4.LineWidth = 1;

     line5.XData = [line5.XData i];
    line5.YData = [line5.YData rand(1)];
    line5.Color = [1 1 0];
    line5.LineWidth = 1;

     line6.XData = [line6.XData i];
    line6.YData = [line6.YData rand(1)];
    line6.Color = [0 1 1];
    line6.LineWidth = 1;
    i=i+1;

    line7.XData = [line7.XData i];
    line7.YData = [line7.YData rand(1)];
    line7.Color = [1 0 0];
    line7.LineWidth = 1;
    
    line8.XData = [line8.XData i];
    line8.YData = [line8.YData rand(1)];
    line8.Color = [1 1 0];
    line8.LineWidth = 1;

    line9.XData = [line9.XData i];
    line9.YData = [line9.YData rand(1)];
    line9.Color = [0 1 0];
    line9.LineWidth = 1;

    line10.XData = [line10.XData i];
    line10.YData = [line10.YData rand(1)];
    line10.Color = [1 1 1];
    line10.LineWidth = 1;

    line11.XData = [line11.XData i];
    line11.YData = [line11.YData rand(1)];
    line11.Color = [1 1 0];
    line11.LineWidth = 1;

    line12.XData = [line12.XData i];
    line12.YData = [line12.YData rand(1)];
    line12.Color = [0 1 1];
    line12.LineWidth = 1;
    
    if i>200

        x_r13 = x_r13(2:end);
        x_r14 = x_r14(2:end);
        x_r15 = x_r15(2:end);
        x_r16 = x_r16(2:end);
        x_r17 = x_r17(2:end);
        
        x_r13(end+1)=rand(1);
        x_r14(end+1)=rand(1);
        x_r15(end+1)=rand(1);
        x_r16(end+1)=rand(1);
        x_r17(end+1)=rand(1);

        line13.XData = (1:100);
        line13.YData = x_r13;
        line14.XData = (1:100);
        line14.YData = x_r14;
        line15.XData = (1:100);
        line15.YData = x_r15;
        line16.XData = (1:100);
        line16.YData = smoothdata(x_r16,20,'sgolay');
        line17.XData = (1:100);
        [b, a] = cheby1(4, 20,(400/(1000/2))); % fs = 1000, fc = 400
        o = filter(b, a, x_r17);
        line17.YData = o;

        dcm_filtered = angle2dcm( rand(1), rand(1), rand(1));
        VR_filtered=dcm_filtered*V;
        XR_filtered=reshape(VR_filtered(1,:),4,6);
        YR_filtered=reshape(VR_filtered(2,:),4,6);
        ZR_filtered=reshape(VR_filtered(3,:),4,6);

        fill3(XR_filtered,YR_filtered,ZR_filtered,C,'FaceAlpha',alpha,'EdgeColor',[0.24,0.24,0.24]);
        xlim([-2 2]);
        ylim([-2 2]);
        zlim([-2 2]);
        box on;

        
%         fuse = complementaryFilter('SampleRate',1000); % fs = 1000
%         q = fuse(x_r14, x_r15);
%         disp(q);

        


    else
        x_l(end+1)=i;

        x_r13(end+1)=rand(1);
        x_r14(end+1)=rand(1);
        x_r15(end+1)=rand(1);
        x_r16(end+1)=rand(1);
        x_r17(end+1)=rand(1);
       

        line13.XData = [line13.XData i];
        line13.YData = [line13.YData rand(1)];
        line14.XData = [line14.XData i];
        line14.YData = [line14.YData rand(1)];
        line15.XData = [line15.XData i];
        line15.YData = [line15.YData rand(1)];
        line16.XData = [line16.XData i];
        line16.YData = [line16.YData rand(1)];
        line17.XData = [line17.XData i];
        line17.YData = [line17.YData rand(1)];
    end

    line13.Color = [0.9 0.1 0.2];
    line13.LineWidth = 1;
    line14.Color = [0 0.3 0.5];
    line14.LineWidth = 1;
    line15.Color = [0 0.9 0.7];
    line15.LineWidth = 1;
    line16.Color = [0.9 0.9 0.7];
    line16.LineWidth = 1;
    line17.Color = [0.5 0.1 0.7];
    line17.LineWidth = 1;

    
    i=i+1;
end