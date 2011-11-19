function [V,A,Omega,Alpha]=getVelocityAcceleration(P,Q,t)

n=size(P,2);
V=zeros(3,n-1);
Omega=zeros(4,n-1);
A=zeros(4,n-2);
Alpha=zeros(3,n-2);

dPdt=diff(P,1,2)./diff(t,1,2);
d2Pdt2=diff(dPdt,1,2)./diff(t(1:end-1),1,2);

dQdt=quaternionderivative(Q,t);
d2Qdt2=quaternionDerivative(dQdt,t(1:end-1));

Qstar=zeros(size(Q));

for i=1:n
    Qstar(:,i)=quaternionconjugate(Q(:,i));
end

for i=1:n-1    
    V(:,i)=quaternionproduct(quaternionproduct(Qstar(:,i),[0;dPdt(:,i)]),Q(:,i));
    Omega(:,i)=2*quaternionproduct(Qstar(:,i),dQdt(:,i));
    if i~=n-1
        A(:,i)=quaternionproduct(quaternionproduct(Qstar(:,i),[0;d2Pdt2(:,i)]),Q(:,i));
        Alpha(:,i)=2*quaternionproduct(Qstar(:,i),d2Qdt2(:,i));
    end    
end

end