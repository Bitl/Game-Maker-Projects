ini_open("biorena_savedwavedata.ini")
iniwave = ini_read_real("wavedata","wave",1)
global.wave = iniwave
ini_close()
