function c = rot(a,adot,b)
    c = cross(a,cross(a,b)) + cross(adot,b);
end