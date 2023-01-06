function [y1, y2] = CombinedCrossover(x1,x2,gamma)

    m = randi([1,3]);
    switch m
        case 1
            [y1, y2] = SinglePointCrossover(x1,x2);
        case 2
            [y1, y2] = DoublePointCrossover(x1,x2);
        otherwise
            [y1, y2] = UniformCrossover(x1,x2,gamma);
    end

end