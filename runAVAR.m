function nop = runAVAR(data, column, Fs, text)

omega = data(:,column);
omega = omega(1:length(omega),:);

t0 = 1/Fs;

theta = cumsum(omega, 1)*t0;

maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.

tau = m*t0;

avar = zeros(numel(m), 1);
for i = 1:numel(m)
    mi = m(i);
    avar(i,:) = sum( ...
        (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
end
avar = avar ./ (2*tau.^2 .* (L - 2*m));

adev = sqrt(avar);

figure
loglog(tau, adev)
title(compose('Allan Deviation - %s', text))
xlabel('\tau');
ylabel('\sigma(\tau)')
grid on
axis equal

end