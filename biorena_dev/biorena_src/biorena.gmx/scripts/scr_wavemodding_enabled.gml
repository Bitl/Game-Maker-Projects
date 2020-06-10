ini_open("biorena_savedwavedata.ini")
inimod = ini_read_real("wavemodding","mod_enabled",0)
global.modding_enabled = inimod
ini_close()
