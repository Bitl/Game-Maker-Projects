ini_open("biorena_savedwavedata.ini")
inibot = ini_read_real("wavemodding","spawn_bot",1)
global.mod_bot_enabled=inibot
ini_close()
