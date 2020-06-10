ini_open("biorena_savedwavedata.ini")
initimeplayer = ini_read_real("wavemodding","mod_respawntime_player",150)
global.player_respawntime = initimeplayer
ini_close()
