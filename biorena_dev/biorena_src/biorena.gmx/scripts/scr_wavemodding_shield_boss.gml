ini_open("biorena_savedwavedata.ini")
inishield = ini_read_real("wavemodding","mod_shield_ammo_boss",5)
global.mod_shield_ammo_boss = inishield
ini_close()
