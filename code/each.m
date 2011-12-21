function [ out ] = each( images, f, args )
%EACH Performs function on each image

    out = cell(1,numel(images));
    
    if ischar(f)
        f = str2func(f);
    end

    for i=1:numel(images)
        if nargin > 2
            out{i} = f(images{i},args{:});
        else
            out{i} = f(images{i});
        end
    end

end

