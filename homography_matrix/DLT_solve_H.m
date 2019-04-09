function H = DLT_solve_H(coords_homo,target_coords_homo)

i = 1;
A1 = [coords_homo(1,i) coords_homo(2,i) 1 0 0 0 -1*coords_homo(1,i)*target_coords_homo(1,i) -1*coords_homo(2,i)*target_coords_homo(1,i)
    0 0 0 coords_homo(1,i) coords_homo(2,i) 1 -1*coords_homo(1,i)*target_coords_homo(2,i) -1*coords_homo(2,i)*target_coords_homo(2,i)];


i = 2;
A2 = [coords_homo(1,i) coords_homo(2,i) 1 0 0 0 -1*coords_homo(1,i)*target_coords_homo(1,i) -1*coords_homo(2,i)*target_coords_homo(1,i)
    0 0 0 coords_homo(1,i) coords_homo(2,i) 1 -1*coords_homo(1,i)*target_coords_homo(2,i) -1*coords_homo(2,i)*target_coords_homo(2,i)];

i = 3;
A3 = [coords_homo(1,i) coords_homo(2,i) 1 0 0 0 -1*coords_homo(1,i)*target_coords_homo(1,i) -1*coords_homo(2,i)*target_coords_homo(1,i)
    0 0 0 coords_homo(1,i) coords_homo(2,i) 1 -1*coords_homo(1,i)*target_coords_homo(2,i) -1*coords_homo(2,i)*target_coords_homo(2,i)];

i = 4;
A4 = [coords_homo(1,i) coords_homo(2,i) 1 0 0 0 -1*coords_homo(1,i)*target_coords_homo(1,i) -1*coords_homo(2,i)*target_coords_homo(1,i)
    0 0 0 coords_homo(1,i) coords_homo(2,i) 1 -1*coords_homo(1,i)*target_coords_homo(2,i) -1*coords_homo(2,i)*target_coords_homo(2,i)];

A = [A1;A2;A3;A4];

vec = [target_coords_homo(1,1), target_coords_homo(2,1), target_coords_homo(1,2), target_coords_homo(2,2), target_coords_homo(1,3), target_coords_homo(2,3), target_coords_homo(1,4), target_coords_homo(2,4)]'



H = inv(A)*vec
h = [H;1];

H = [h(1), h(2), h(3)
    h(4), h(5), h(6)
    h(7), h(8), h(9)];

end


