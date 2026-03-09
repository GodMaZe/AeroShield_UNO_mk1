function fullpath = getfilename(dirpath, filename, datestr, ext)
    arguments (Input)
        dirpath;
        filename;
        datestr;
        ext = "csv"
    end

    arguments (Output)
        fullpath;
    end
    
    fullpath = "./" + dirpath + "/" + filename + "_" + datestr + "." + ext;
end