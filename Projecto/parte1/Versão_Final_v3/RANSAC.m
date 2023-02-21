function [u1aux,v1aux,u2aux,v2aux] = RANSAC(u1,v1,u2,v2,xyz1,xyz2,ind1,ind2)
ranIndBest=[0 0];
ranInd=[0 0];
[~,nmatches]=size(u1);

%RANSAC
nRansacIterations = 3000;
nRansacPontos = 4;
for l=1:nRansacIterations
    r = randi([1 nmatches],1,5);
    u1aux=0;v1aux=0;u2aux=0;v2aux=0;
    ind1aux=0; ind2aux=0;
    for i=1:nRansacPontos
        u1aux(i) = u1(r(i));
        v1aux(i) = v1(r(i));
        u2aux(i) = u2(r(i));
        v2aux(i) = v2(r(i));
    end
%     figure();imagesc(imaux1);hold on;plot(u1aux,v1aux,'*r');hold off;
%     figure();imagesc(imaux2);hold on;plot(u2aux,v2aux,'*r');hold off;
    ind1aux=sub2ind([480 640],uint64(v1aux),uint64(u1aux));
    ind2aux=sub2ind([480 640],uint64(v2aux),uint64(u2aux));
    %Calculo Matriz Rotacao 1 2
    cent1=mean(xyz1(ind1aux,:))';
    cent2=mean(xyz2(ind2aux,:))';
    pc1=xyz1(ind1aux,:)'-repmat(cent1,1,4);
    pc2=xyz2(ind2aux,:)'-repmat(cent2,1,4);
    [a b c]=svd(pc2*pc1');
    R12=a*c';
    %Juntar Pointclouds 1 2
    xyzt1=R12*(xyz1(ind1,:)'-repmat(cent1,1,length(xyz1(ind1,:))));%Estou a juntar a pointCould1 à pointCloud2 no ref 2!
    xyzt2=xyz2(ind2,:)'-repmat(cent2,1,length(xyz2(ind2,:)));
    
    
    j=1;
    for i=1:nmatches
       dif = abs(norm(xyzt1(:,i))-norm(xyzt2(:,i)));
        if(dif<0.002)
           ranInd(j)=i;
           j=j+1;
        end
    end
    [~, s1]=size(ranInd);
    [~, s2]=size(ranIndBest);
    if(s1>s2)
        ranIndBest=ranInd;
        R12Best = R12;
        
    end
end

%%
[~, s1]=size(ranIndBest);
s1
u1aux=0; v1aux=0; u2aux=0; v2aux=0;
ind1aux=0; ind2aux=0;     
for i=1:s1
        u1aux(i) = u1(ranIndBest(i));
        v1aux(i) = v1(ranIndBest(i));
        u2aux(i) = u2(ranIndBest(i));
        v2aux(i) = v2(ranIndBest(i));
end

end