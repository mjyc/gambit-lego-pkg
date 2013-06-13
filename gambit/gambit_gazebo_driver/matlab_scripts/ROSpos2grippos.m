function outval=ROSpos2grippos(pos)


x = -1.045:0.001:0.785;
temp = zeros(size(x));

for i=1:length(x)
    r = 0.015;
    l = 0.030;
    t = pi/2.0 - x(i);
    temp(i) = r * cos(t) + sqrt(l*l - power(r * sin(t), 2.0));
end

temp = [-Inf, temp, Inf];
for i=1:length(temp)-1
    if pos >= temp(i) && pos < temp(i+1)
        if i==1
            outval=x(1);
        elseif i==length(temp)-1
            outval=x(end);
        else
            outval=x(i-1);
        end
    end
end


% idx = (pos + 1.045)*1000;
% if idx > length(x)-1
%     idx = length(x);
% elseif idx < 0
%     idx = 1;
% else
%     idx = idx + 1;
% end
% outval = temp(idx);

end