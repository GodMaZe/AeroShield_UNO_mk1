function frmt = getdataformat(data,nformat,divider,include_newline)
%GETDATAFORMAT Return the formatting string for the data vector to be
%printted into the console or file.
frmt = join(repmat(nformat, 1, numel(data)), divider);
if nargin>3 && include_newline
    frmt = frmt + "\n";
end
end

