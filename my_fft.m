function [f_P1,f] = my_fft(X,Fs)

    L = size(X,1);

    f_x = fft(X(:,1));
    f_P2 = abs(f_x/L);
    f_P1 = f_P2(1:L/2+1);
    f = Fs*(0:(L/2))/L;
end

