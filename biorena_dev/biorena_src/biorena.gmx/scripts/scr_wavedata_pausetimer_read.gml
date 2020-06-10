ini_open("biorena_savedwavedata.ini")
initimer = ini_read_real("wavemodding","mod_revive_timerpause",0)
global.revive_pausetimer = initimer
ini_close()
