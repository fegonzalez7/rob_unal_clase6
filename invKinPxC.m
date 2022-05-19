%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Cinematica inversa para robot Phantom X
Por: Felipe Gonzalez Roldan
Entradas:
T: Pose del EF
l: Longitud de eslabones
Salida:
q_inv: Variables del espacio articular
4 soluciones 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}
function q_inv = invKinPxC(varargin)
switch nargin
   case 2
      T = varargin{1};
      l = varargin{2};
      %R_p = (rotz(atan2(T(2,4),T(1,4))))'*T(1:3,1:3);
      %pitch = atan2(-R_p(3,1),R_p(1,1));
   case 3
      T = varargin{1};
      l = varargin{2};
      qlim = varargin{3};
      %R_p = (rotz(atan2(T(2,4),T(1,4))))'*T(1:3,1:3);
      %pitch = atan2(-R_p(3,1),R_p(1,1));
   otherwise
      disp('Wrong number of inputs')
end

%a = T(1:3,3);
%x = T(1,4);
%y = T(2,4);
%z = T(3,4);
% Wrist
%Pw = [x y z]'-l(4)*a;
Pw = T(1:3,4)-l(4)*T(1:3,3);

% Solucion para q1
q1a = atan2(T(2,4), T(1,4));
q1b = atan2(-T(2,4),-T(1,4));

% Usando q1a 
% Plano articulaciones 2 - 3
pxy = sqrt(Pw(1)^2 + Pw(2)^2);
z = Pw(3) - l(1);
r = sqrt(pxy^2 + z^2);

% Mecanismo 2R
the3 = acos((r^2 - l(2)^2 -l(3)^2)/(2*l(2)*l(3)));
if isreal(the3)
   alp = atan2(z,pxy);
   the2a = alp - atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));
   the2b = alp + atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));
   
   % Codo Abajo
   q2a =  -pi/2 + the2a; % q2
   q3a =  the3; % q3
   
   % Codo Arriba
   q2b =  -pi/2 + the2b;
   q3b = -the3;
   
   % Orientacion
   R_p = (rotz(q1a))'*T(1:3,1:3);
   pitch = atan2(R_p(3,1),R_p(1,1));
   % Codo Abajo
   q4a = pitch - (q2a + q3a); % q4
   % Codo Arriba
   q4b = pitch - (q2b + q3b);
else
   q2a = NaN;
   q3a = NaN;
   q4a = NaN;
   q2b = NaN;
   q3b = NaN;
   q4b = NaN;
end
q_inv(1,1:4) = [q1a q2a q3a q4a];
q_inv(2,1:4) = [q1a q2b q3b q4b];

% Queda invertido el yaw debido a la restriccion de movimiento
% Usando q1b
q_inv(3,1:4) = [q1b -q2a -q3a -q4a];
q_inv(4,1:4) = [q1b -q2b -q3b -q4b];

for i=1:size(q_inv,1)
   if any(isnan(q_inv(i,:)))
      q_inv(i,:) = [NaN NaN NaN NaN];
   end
end
