% Rectangular to spherical coord function
% Returns a 3x3 Matrix of rectangular coords from spherical coords
function ShperToRec = ShperToRec(a,i)
    ShperToRec = [cos(a)*cos(i), -sin(a), -cos(a)*sin(i);
                  sin(a)*cos(i), cos(a), -sin(a)*sin(i);
                     sin(i),        0,        cos(i)  ];
end