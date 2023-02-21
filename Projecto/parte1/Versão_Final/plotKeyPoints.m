function plotKeyPoints(F)
    perm = randperm(size(F,2)) ;
    sel = perm(1:91) ;
    h1 = vl_plotframe(F(:,sel)) ;
    h2 = vl_plotframe(F(:,sel)) ;
    set(h1,'color','k','linewidth',3) ;
    set(h2,'color','y','linewidth',2) ;
end