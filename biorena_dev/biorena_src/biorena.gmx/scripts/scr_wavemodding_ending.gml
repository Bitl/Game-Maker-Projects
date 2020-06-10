ini_open("biorena_savedwavedata.ini")
iniending = ini_read_real("wavemodding","mod_ending",0)
global.mod_end = iniending
ini_close()
