ini_open("biorena_savedwavedata.ini")
inishield = ini_read_real("wavemodding","mod_shield_ammo",3)
global.mod_shield_ammo = inishield
ini_close()
