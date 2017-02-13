XID = fopen('PointsCloudX.txt','r');
x = fscanf(XID,'%*s\n%f',300);
fclose(XID);
YID = fopen('PointsCloudY.txt','r');
y = fscanf(YID,'%*s\n%f',300);
fclose(YID);
ZID = fopen('PointsCloudZ.txt','r');
z = fscanf(ZID,'%*s\n%f',300);
fclose(ZID);

cx=1.001;
cy=0.004;
cz=-0.37;
figure(1);
scatter3(x,y,z,10);
hold on;
%scatter3(cx,cy,cz,10,[1,1,0]);