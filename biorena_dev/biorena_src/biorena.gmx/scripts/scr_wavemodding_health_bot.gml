ini_open("biorena_savedwavedata.ini")
inihealth = ini_read_real("wavemodding","mod_health_bot",200)
global.mod_bot_hp = inihealth
ini_close()
