function [gyro, bias] = gyro_model(w, bias, p)
% gyro = true rate + bias + white noise, bias does a small random walk
bias = bias + p.bias_rw * sqrt(p.dt) * randn(3,1);
gyro = w + bias + p.sig_gyro * randn(3,1);
end
