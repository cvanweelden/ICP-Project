function [ out ] = each( elements, f, args )
%EACH Performs function on each element

    out = cell(1,numel(elements));
    
    if ischar(f)
        f = str2func(f);
    end

    for i=1:numel(elements)
        if nargin > 2
            out{i} = f(elements{i},args{:});
        else
            out{i} = f(elements{i});
        end
    end

end

