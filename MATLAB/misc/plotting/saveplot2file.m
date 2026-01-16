function saveplot2file(fig, name, aspdf)
    arguments (Input)
        fig;
        name;
        aspdf = false;
    end
    if numel(name) == 0
        return;
    end

    if aspdf
        saveas(fig, name, "pdf");
    else
        saveas(fig, name, "svg");
    end
end