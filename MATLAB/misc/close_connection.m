function close_connection(scon, dfile_handle, input)
%close_connection Method used after the measurement is finished to close
%all the necessary opened connections and timers.

arguments (Input)
    scon;
    dfile_handle
    input = 0;
end

if exist("scon", "var")
    fprintf("Closing serial communication...\n");
    write(scon, input, 'single');
    scon.delete()
end
if exist("dfile_handle", "var")
    fprintf("Closing file stream...\n");
    fclose(dfile_handle);
    clear dfile_handle;
end
for tim=timerfindall
    fprintf("Closing timer thread: %s...\n", tim.Name);
    stop(tim);
    delete(tim);
end
end