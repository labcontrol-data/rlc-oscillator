% procedure to compute LRAC in the deadbeat paper

function [custoLRAC] = computeLRAC_Deadbeat(A,B,G,H,h,Nit)

Nit=max(Nit,250);

Acl = (eye(2,2)+h*A) + h*B*G;

X=eye(2,2);
for k=1:Nit
    X = Acl*X*(Acl') + H*H';
end

Q = [1 + (G(1)-0.1)*G(2) + 0.1*G(2)^2*exp(-G(1))  0;
       0             0.02*(G(1)-4)^2+ abs(G(1)*G(2)) + 0.1];

custoLRAC=trace(Q*X);
