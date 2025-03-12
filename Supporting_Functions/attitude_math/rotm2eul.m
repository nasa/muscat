function eul = rotm2eul(rotm)
eul = zeros(1,3);
eul(3) = atan2(rotm(3,2),rotm(3,3));
eul(2) = atan2(-rotm(3,1),sqrt(rotm(3,2)^2+rotm(3,3)^2));
eul(1) = atan2(rotm(2,1),rotm(1,1));
end