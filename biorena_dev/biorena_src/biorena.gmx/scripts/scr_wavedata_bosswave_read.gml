ini_open("biorena_savedwavedata.ini")
iniboss = ini_read_real("wavedata","bosswave",0)
global.bosswave = iniboss
ini_close()
