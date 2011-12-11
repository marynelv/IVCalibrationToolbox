function handleP = DisplayCameraPlane(C, R, windowScale)

R = R';
window11 = windowScale*[1;1;1.5]; window12 = windowScale*[-1; 1; 1.5];   window21 = windowScale*[1;-1;1.5];    window22 = windowScale*[-1; -1; 1.5];
windowPrime11 = R*window11+C; windowPrime12 = R*window12+C; windowPrime21 = R*window21+C; windowPrime22 = R*window22+C;

hold on
plot3([C(1) C(1)+windowScale*R(1,2)], [C(2) C(2)+windowScale*R(2,2)], [C(3) C(3)+windowScale*R(3,2)], 'g-');
hold on
plot3([C(1) C(1)+windowScale*R(1,1)], [C(2) C(2)+windowScale*R(2,1)], [C(3) C(3)+windowScale*R(3,1)], 'r-');

hold on
plot3([windowPrime11(1),C(1),windowPrime22(1)], ...
      [windowPrime11(2),C(2),windowPrime22(2)], ...
      [windowPrime11(3),C(3),windowPrime22(3)], 'k-');
hold on
plot3([windowPrime12(1),C(1),windowPrime21(1)], ...
      [windowPrime12(2),C(2),windowPrime21(2)], ...
      [windowPrime12(3),C(3),windowPrime21(3)], 'k-');
hold on,
patchcoord = [windowPrime11, windowPrime12, windowPrime22, windowPrime21, windowPrime11];
hold on, patch(patchcoord(1,:)', patchcoord(2,:)', patchcoord(3,:)', 'k'), alpha(.3), hold on
plot3([windowPrime11(1), windowPrime12(1), windowPrime22(1), windowPrime21(1), windowPrime11(1)],...
                [windowPrime11(2), windowPrime12(2), windowPrime22(2), windowPrime21(2), windowPrime11(2)], ...
                [windowPrime11(3), windowPrime12(3), windowPrime22(3), windowPrime21(3), windowPrime11(3)], 'k-');
