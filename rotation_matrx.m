function shifts=rotation_matrx()
shifts=[[0 0 0];[0,-0.1,0];[-0.075,-0.175,pi/2];[-0.175,-0.175,pi/2];[-0.25,-0.1,0];[-0.25,0,0];[-0.175,0.075,pi/2];[-0.075,0.075,pi/2]];
X=shifts(:,1);
Y=shifts(:,2);
Z=shifts(:,3);
Centre=[-0.1250,-0.0500]
pgon=polyshape(X,Y);
plot(pgon);
hold on
th=4*pi/180;
for i=1:30
    for j=1:length(X)
        a=X(j);
        b=Y(j);
        X(j)=(-Centre(1)+a)*cos(th)-(-Centre(2)+b)*sin(th);
        Y(j)=(-Centre(1)+a)*sin(th)+(-Centre(2)+b)*cos(th);
        Z(j)=Z(j)+th;
        X(j)=X(j)+Centre(1);
        Y(j)=Y(j)+Centre(2);
        shifts(end+1,:)=[X(j),Y(j),Z(j)];
    end
    pgon=polyshape(X,Y);
    plot(pgon);
    hold on
end
end
