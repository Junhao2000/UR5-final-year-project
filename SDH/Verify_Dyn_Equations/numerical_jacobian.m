% 用于计算给定向量值函数f关于自变量向量x的雅各比矩阵。函数使用有限差分法（finite difference method）来计算偏导数。
% 输入：f：向量值函数，表示关节角、关节角速度和关节扭矩之间的关系。 x：自变量向量，例如机械臂的状态向量。h：差分步长，用于计算有限差分。
% 输出：f关于x的雅各比矩阵J
% 这个实现在计算雅各比矩阵时避免了符号计算，从而加速计算过程
function J = numerical_jacobian(f, x, h)
    n = length(x);
    m = length(f(x));
    J = zeros(m, n);
    
    for i = 1:n
        x_plus = x;
        x_plus(i) = x_plus(i) + h;
        x_minus = x;
        x_minus(i) = x_minus(i) - h;
        
        f_plus = f(x_plus);
        f_minus = f(x_minus);
        
        J(:, i) = (f_plus - f_minus) / (2 * h);
    end
end