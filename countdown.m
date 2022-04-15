function countdown(message, seconds)
for i = flip(1:seconds)
    clc;
    fprintf("%s\t%d\n", message, i);
    pause(1);
end
clc