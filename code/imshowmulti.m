function imshowmulti( images, titles, sep )
%IMSHOWMULTI Shows a list (cell) of images

    w = floor(sqrt(numel(images))*1.5);
    h = ceil(numel(images)/w);
    
    if nargin > 2 && strcmpi(sep,'separate')
        for i = 1:numel(images)
            figure;
            imshow(images{i},[])
        end
    else
        figure;
        for i=1:numel(images)
            subplot(h,w,i)
            imshow(images{i},[])
            if nargin > 1
                title(titles(i));
            end
        end
    
    end

end

