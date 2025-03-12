function rotm = eul2rotm(eul)
rotm = rotz(eul(1))*roty(eul(2))*rotx(eul(3));
end