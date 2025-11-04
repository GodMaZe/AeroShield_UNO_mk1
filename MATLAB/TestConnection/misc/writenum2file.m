function writenum2file(filehandle, data, verbose, Ts, dt)
%WRITE2FILE Write data in csv format.
%   Write the measured data into a file in csv format, while
%being able to display the measurement within the console, indicating
%stability of the sampling period.
fprintf(filehandle, getdataformat(data,"%8.3f",",",1), data);

if nargin > 2
    if verbose
        frmt = getdataformat(data, "%8.3f"," ");
        if nargin > 4
            if (dt > (Ts*1.05))
                fprintf(frmt + " --\n", data);
            else
                fprintf(frmt + "\n", data);
            end
        else
            fprintf(frmt + "\n", data);
        end
    end
end
end

