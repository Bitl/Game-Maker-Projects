ini_open("biorena_savedwavedata.ini")
inihealth = ini_read_real("wavemodding","mod_health",200)
global.mod_hp = inihealth
ini_close()
