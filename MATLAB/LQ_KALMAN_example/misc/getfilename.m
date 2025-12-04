function fullpath = getfilename(dirpath, filename, datestr, ext)
    if nargin < 3
        error("At least the first 3 parameters need to be provided.");
    end
    if nargin == 3
        ext = "csv";
    end

    fullpath = "./" + dirpath + "/" + filename + "_" + datestr + "." + ext;
end