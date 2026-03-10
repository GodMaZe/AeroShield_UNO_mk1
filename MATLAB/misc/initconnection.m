% Open the CSV file for writing
if(exist("dfile_handle", "var"))
    fclose(dfile_handle);
    clear dfile_handle;
end

dfile_handle = fopen(FILEPATH, 'w');
fprintf(dfile_handle, join(OUTPUT_NAMES,",") + "\n");

% Open the serial port for communication with the Arduino (system)
if(exist("scon", "var"))
    scon.flush("input");
    clear scon;
end

loadconfigs;

scon = serialport(COMPORT, CF_BAUDRATE, "Timeout", CF_TIMEOUT);

sline = "";

while(~contains(sline, "MCU"))
    sline = readline(scon);
end

disp(sline);