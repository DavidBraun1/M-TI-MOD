function out = qadd(x, y)
    out = x(:) + y(:);
end

function out = qnorm(x)
    out = sqrt(sum(x.^2));
end

function out = qconj(x)
    out = x(2:4) .* -1;
end

function out = qsmult(a, x)
    out = a .* x;
end

function out = qnormalize(x)
    out = qsmult(1/qnorm(x), x);
end

function out = qmult(x, y)
    p = [0 0 0 0];
    for i=1:5
        for j=1:5
            
        end
    end
    out = p;
end

x = [1, 2, -4, 5];
y = [-5,-1,3,2];
a = 5;

add = qadd(x, y)
norm = qnorm(x)
conj = qconj(x)
smult = qsmult(a, x)
normallize = qnormalize(x)
