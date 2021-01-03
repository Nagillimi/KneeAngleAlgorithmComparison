function c = rotT(a,adot,b)
    c = cross(cross(b,a),a) + cross(b,adot);
end