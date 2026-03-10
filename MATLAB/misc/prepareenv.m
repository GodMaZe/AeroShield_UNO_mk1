function [fPath,fPathMat,oNames] = prepareenv(fileName, csvHeaderString, dirName)
%PREPAREENV Function used for preparing the measuring environment.
%   This function will automatically create the directory, necessary files
%   including MAT file. Split the CSV header string containing the names of
%   the columns in the following format: "col1,col2,col3,...,colN" to an array of column names.
arguments (Input)
    fileName;
    csvHeaderString = [];
    dirName = "dataRepo";
end

arguments (Output)
    fPath;
    fPathMat;
    oNames;
end

DDIR = dirName;
FILENAME = fileName;

if ~exist(DDIR, "dir")
    fprintf("Creating the [%s] data repository folder, for saving the measurements...\n", DDIR);
    mkdir(DDIR);
end

dateString = convertCharsToStrings(datestr(datetime('now'), "yyyy_mm_dd_HH_MM_ss"));

% File paths for the csv and mat files.
fPath = getfilename(DDIR, FILENAME, dateString);
fPathMat = getfilename(DDIR, FILENAME, dateString, 'mat');

% The names of the parameters to write into the file
if isempty(csvHeaderString)
    oNames = [];
else
    oNames = split(csvHeaderString, ',');
end
end