ini_open("biorena_savedwavedata.ini")
initimebot = ini_read_real("wavemodding","mod_respawntime_bot",150)
global.bot_respawntime = initimebot
ini_close()
