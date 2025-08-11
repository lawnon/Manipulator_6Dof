function [signal] = ToSgn(real)
signal = real/abs(real);
end