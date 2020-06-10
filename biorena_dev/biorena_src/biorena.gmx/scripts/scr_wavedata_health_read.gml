ini_open("biorena_savedwavedata.ini")
inihealth = ini_read_real("wavedata","health",200)
global.hp = inihealth
ini_close()
