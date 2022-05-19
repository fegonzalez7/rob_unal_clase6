%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Cinematica inversa para robot SCARA
Por: Felipe Gonzalez Roldan
Entradas: 
T: MTH de entrada
l: Dimensiones de los eslabones
codo: +1 o -1 para solucion del codo
Salida:
q_inv_scara: Variables del espacio articular solución para T
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}
function q_inv_scara = inv_scara(T,l,codo)
x = T(1,4);
y = T(2,4);
z = T(3,4);
r = sqrt(x^2 + y^2);
D = (r^2 - l(2)^2 - l(3)^2)/(2*l(2)*l(3));
RPY = tr2rpy(T);
q3 = l(1) - l(4) - z;
q2 =  atan2(codo*sqrt(1-D^2),D);
q1 = atan2(y,x) - atan2(l(3)*sin(q2),l(3)*cos(q2)+l(2));
q4 = q1 + q2 + RPY(3);
q_inv_scara = [q1 q2 q3 q4];
