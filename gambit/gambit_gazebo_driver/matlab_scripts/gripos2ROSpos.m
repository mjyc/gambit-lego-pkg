function ROSpos=gripos2ROSpos(pos)

r = 0.015;
l = 0.030;
t = pi/2.0 - pos;
ROSpos = r * cos(t) + sqrt(l*l - power(r * sin(t), 2.0));

end