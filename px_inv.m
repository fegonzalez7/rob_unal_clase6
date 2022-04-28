l = [14.5, 10.7, 10.7, 9]; % Longitudes eslabones
% Definicion del robot RTB
L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,   'a',l(2),'d',0,   'offset',pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,   'a',l(3),'d',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,   'a',0,   'd',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L,'name','Px');
% roty(pi/2)*rotz(-pi/2)
PhantomX.tool = [0 0 1 l(4); -1 0 0 0; 0 -1 0 0; 0 0 0 1];
ws = [-50 50];
% Graficar robot
q = deg2rad([60, -50, -50, 10]);
PhantomX.plot(q,'notiles','noname');
hold on
trplot(eye(4),'rgb','arrow','length',15,'frame','0')
axis([repmat(ws,1,2) 0 60])
PhantomX.teach()
%%
M = eye(4);
for i=1:PhantomX.n
    M = M * L(i).A(q(i));
    trplot(M,'rgb','arrow','frame',num2str(i),'length',15)
end
%%
qt = deg2rad([60 -80 20 25]);
Tt = PhantomX.fkine(qt);
%%
% Desacople
T = Tt;
Tw = T - l(4)*T(1:4,3); % MTH Wrist

% Solucion q1
q1 = atan2(Tw(2,4),Tw(1,4));
%rad2deg(q1)

% Solución 2R
h = Tw(3,4) - l(1);
r = sqrt(Tw(1,4)^2 + Tw(2,4)^2);

% Codo abajo
the3 = acos((r^2+h^2-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
the2 = atan2(h,r) - atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));

q2d = -(pi/2-the2);
q3d = the3;

% Codo arriba
% the3 = acos((r^2+h^2-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
% the2 = atan2(h,r) + atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));
q2u = -(pi/2-the2);
q3u = -the3;

%disp(rad2deg([q2d q3d; q2u q3u]))

% Solucion de q4
Rp = (rotz(q1))'*T(1:3,1:3);
pitch = atan2(Rp(3,1),Rp(1,1));

q4d = pitch - q2d - q3d;
q4u = pitch - q2u - q3u;
%disp(rad2deg([q4d q4u]))

qinv(1,1:4) = [q1 q2u q3u q4u];
qinv(2,1:4) = [q1 q2d q3d q4d];
disp(rad2deg(qinv))