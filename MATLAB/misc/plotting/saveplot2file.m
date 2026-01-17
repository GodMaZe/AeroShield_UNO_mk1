function saveplot2file(fig, name, aspdf)
    arguments (Input)
        fig;
        name;
        aspdf = false;
    end
    if numel(name) == 0
        return;
    end

    pathtoimg = split(name, ["/","\"]);
    
    % Ensure the directory exists before saving the file
    if ~exist(strjoin(pathtoimg(1:end-1), '/'), 'dir')
        mkdir(strjoin(pathtoimg(1:end-1), '/'));
    end

    if aspdf
        saveas(fig, name, "pdf");
    else
        saveas(fig, name, "svg");
    end
end