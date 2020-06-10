ini_open("biorena_savedwavedata.ini")
iniboss = ini_read_real("wavemodding","mod_boss_spawnvalue",11)
global.mod_bosswavespawnvalue = iniboss
ini_close()
