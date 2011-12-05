function [V,A,Omega,Alpha,Vw]=getVelocityAcceleration(P,Q,t,gravityvec)

n=size(P,2);
V=zeros(3,n-1);
Omega=zeros(3,n-1);
A=zeros(3,n-2);
Alpha=zeros(3,n-2);

dPdt=bsxfun(@rdivide,diff(P,1,2),diff(t,1,2));
d2Pdt2=bsxfun(@rdivide,diff(dPdt,1,2),diff(t(1:end-1),1,2));
Qstar=zeros(size(Q));
for i=1:n
    Qstar(:,i)=quaternionconjugate(Q(:,i));
end

dQdt=quaternionDerivative(Q,t);
dQdtstar=quaternionDerivative(Qstar,t);
d2Qdt2=quaternionDerivative(dQdt,t(1:end-1));


for i=1:n-1
    v=quaternionproduct(quaternionproduct(Qstar(:,i),[0;dPdt(:,i)]),Q(:,i));
    V(:,i)=v(2:end);
    om=2*quaternionproduct(Qstar(:,i),dQdt(:,i));
    Omega(:,i)=om(2:end);
    if i~=n-1
%         C_q_world_IMU = quaternion2matrix(Q(:,i));
%         aprime= C_q_world_IMU(1:3,1:3)'*(d2Pdt2(:,i)-gravityvec);
        a=quaternionproduct(quaternionproduct(Qstar(:,i),[0;d2Pdt2(:,i)-gravityvec]),Q(:,i));
        A(:,i)=a(2:end);
        al=2*quaternionproduct(dQdtstar(:,i),dQdt(:,i))+2*quaternionproduct(Qstar(:,i),d2Qdt2(:,i));
        Alpha(:,i)=al(2:end);
    end
end

V=[V,V(:,end)];
Vw=[dPdt,dPdt(:,end)];
Omega=[Omega,Omega(:,end)];
A=[A,A(:,end),A(:,end)];
Alpha=[Alpha,Alpha(:,end),Alpha(:,end)];

end