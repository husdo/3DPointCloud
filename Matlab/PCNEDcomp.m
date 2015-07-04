function pcNED=PCNEDcomp(range,angleaz,angleEl,tNED,psi,teta,phi)

R_NED2SRF=R_by_EA(phi*pi/180,teta*pi/180,psi*pi/180);

raysdirappo=[0 0 1];
raysdir=zeros(size(angleaz,1),3);

for i=1:size(angleaz,1)
    Rot=R_by_EA(-angleaz(i)*pi/180,angleEl,0);
    
    raysdir(i,:)=(Rot*raysdirappo')';
end
pcSRF=[raysdir(:,1).*range(1,:)' raysdir(:,2).*range(1,:)' raysdir(:,3).*range(1,:)'];
pcBRF=[pcSRF(:,3) pcSRF(:,2) -pcSRF(:,1)];


pcNED=(R_NED2SRF'*pcBRF')'+repmat(tNED,size(angleaz,1),1);
end