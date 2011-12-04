function Pout=bezierCurve(P,numpts)

n=size(P,2)-1;
delta=1/(numpts-1);

t=0:delta:1;

Basis=zeros(n+1,length(t));

for i=0:n
Basis(i+1,:)=nchoosek(n,i) .* (t.^i) .* (1-t).^(n-i);
end

Pout=P*Basis;

end