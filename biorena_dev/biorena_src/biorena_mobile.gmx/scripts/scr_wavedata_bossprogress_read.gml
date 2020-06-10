ini_open("biorena_savedwavedata.ini")
iniprogress=ini_read_real("wavedata","boss_wavespawnprogress",1)
global.wavebossspawn = iniprogress
ini_close()
