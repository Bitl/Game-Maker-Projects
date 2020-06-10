ini_open("biorena_savedwavedata.ini")
inilevel = ini_read_real("wavemodding","mod_revive_revivallevel",35)
global.revive_maxrevivallevel = inilevel
ini_close()
