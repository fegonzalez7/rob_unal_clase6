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
q = deg2rad([60, -70, 20, 15]);
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
qt = deg2rad([60, -70, 20, 15]);
Tt = PhantomX.fkine(qt);
%%
% Desacople
T = Tt;
Posw = T(1:3,4) - l(4)*T(1:3,3); % MTH Wrist
%%
% Solucion q1
q1 = atan2(Posw(2),Posw(1));
rad2deg(q1)
%%
% Solución 2R
h = Posw(3) - l(1);
r = sqrt(Posw(1)^2 + Posw(2)^2);

% Codo abajo
the3 = acos((r^2+h^2-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
the2 = atan2(h,r) - atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));

q2d = -(pi/2-the2);
q3d = the3;

% Codo arriba
% the3 = acos((r^2+h^2-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
the2 = atan2(h,r) + atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));
q2u = -(pi/2-the2);
q3u = -the3;

disp(rad2deg([q2d q3d; q2u q3u]))
%%
% Solucion de q4
Rp = (rotz(q1))'*T(1:3,1:3);
pitch = atan2(Rp(3,1),Rp(1,1));

q4d = pitch - q2d - q3d;
q4u = pitch - q2u - q3u;
%disp(rad2deg([q4d q4u]))

qinv(1,1:4) = [q1 q2u q3u q4u];
qinv(2,1:4) = [q1 q2d q3d q4d];
disp(rad2deg(qinv))
%%
% Solucion alternativa para q4
T03d = PhantomX.A([1 2 3],[q1 q2d q3d]);
R_3Td = (T03d(1:3,1:3))'*T(1:3,1:3);
% syms q4
% L(4).A(q4)*PhantomX.tool
% [ sin(q4),  0, cos(q4), 9*cos(q4)]
% [-cos(q4),  0, sin(q4), 9*sin(q4)]
% [       0, -1,       0,         0]
% [       0,  0,       0,         1]
q4d = atan2(R_3Td(1,1),-R_3Td(2,1));
rad2deg(q4d)
%%
T03u = PhantomX.A([1 2 3],[q1 q2u q3u]);
R_3Tu = (T03u(1:3,1:3))'*T(1:3,1:3);
q4u = atan2(R_3Tu(1,1),-R_3Tu(2,1));
qinv(1,1:4) = [q1 q2u q3u q4u];
qinv(2,1:4) = [q1 q2d q3d q4d];
disp(rad2deg(qinv))
%%
T0 = transl(15,10,18)*trotz(pi/6)*troty(pi/2);
T1 = transl(15,-10,18)*trotz(-pi/6)*troty(pi/2);
% ctraj
T01 = ctraj(T0,T1,20);
% ciclo para calcular y graficar el robot
pause(3)
for i=1:20
   qinv = invKinPxC(T01(:,:,i),l);
   PhantomX.plot(qinv(2,:),'notiles','noname')
   plot3(T01(1,4,i),T01(2,4,i),T01(3,4,i),'ro')
   q_inv(i,:) = qinv(2,:);
   %pause(0.5)
end
%%
figure(2)
plot(q_inv(:,:),'lineWidth',2)
grid on
axis([1 20 -pi pi])

