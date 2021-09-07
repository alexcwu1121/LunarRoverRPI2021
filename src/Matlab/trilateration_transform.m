function [h,h_trans]=trilateration_transform(r_pos)
    syms x y xi yi
    d=sqrt((x-xi)^2+(y-yi)^2);
    %x_partial=(x-xi)/sqrt((x-xi)^2+(y-yi)^2);
    %y_partial=(y-yi)/sqrt((x-xi)^2+(y-yi)^2);
    
    h=[];
    h_trans=[];
    for i=1:size(r_pos,1)
        h=[h;subs(d,[xi yi],[r_pos(i,1) r_pos(i,2)])];
        h_trans=[h_trans;
            subs(diff(d,x),[xi yi],[r_pos(i,1) r_pos(i,2)]), subs(diff(d,y),[xi yi],[r_pos(i,1) r_pos(i,2)]),0,0];
    end
end