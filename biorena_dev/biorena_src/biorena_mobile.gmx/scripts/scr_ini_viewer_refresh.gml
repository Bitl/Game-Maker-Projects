ini_open("biorena_savedwavedata.ini")
menuText[0] = 31
menuText[1] = "PLAYER INTIAL HEALTH: "+string(ini_read_real("wavedata","health",200))
menuText[2] = "BOT INTIAL HEALTH: "+string(ini_read_real("wavedata","health_bot",200))
menuText[3] = "WAVE: "+string(ini_read_real("wavedata","wave",1))
menuText[4] = "BOHROK COUNT: "+string(ini_read_real("wavedata","bohrok",5))
menuText[5] = "INFECTED COUNT: "+string(ini_read_real("wavedata","infected",4))
menuText[6] = "RAHKSHI COUNT: "+string(ini_read_real("wavedata","rahkshi",2))
menuText[7] = "MAKUTA COUNT: "+string(ini_read_real("wavedata","makuta",1))
menuText[8] = "BOHROK SPAWNER COUNT: "+string(ini_read_real("wavedata","bohrok_spawner",5))
menuText[9] = "INFECTED SPAWNER COUNT: "+string(ini_read_real("wavedata","infected_spawner",4))
menuText[10] = "RAHKSHI SPAWNER COUNT: "+string(ini_read_real("wavedata","rahkshi_spawner",2))
menuText[11] = "MAKUTA SPAWNER COUNT: "+string(ini_read_real("wavedata","makuta_spawner",1))
menuText[12] = "BOSSWAVE: "+string(ini_read_real("wavedata","bosswave",0))
menuText[13] = "WAVE PROGRESS: "+string(ini_read_real("wavedata","boss_wavespawnprogress",1))
menuText[14] = "MODDING ENABLED: "+string(ini_read_real("wavemodding","mod_enabled",0))
menuText[15] = "BOT ENABLED: "+string(ini_read_real("wavemodding","spawn_bot",1))
menuText[16] = "PLAYER RESPAWN HEALTH: "+string(ini_read_real("wavemodding","mod_health",200))
menuText[17] = "BOT RESPAWN HEALTH: "+string(ini_read_real("wavemodding","mod_health_bot",200))
menuText[18] = "BOHROK RESET VALUE: "+string(ini_read_real("wavemodding","mod_bohrok_reset",5))
menuText[19] = "INFECTED RESET VALUE: "+string(ini_read_real("wavemodding","mod_infected_reset",4))
menuText[20] = "RAHKSHI RESET VALUE: "+string(ini_read_real("wavemodding","mod_rahkshi_reset",2))
menuText[21] = "MAKUTA RESET VALUE: "+string(ini_read_real("wavemodding","mod_makuta_reset",1))
menuText[22] = "WAVE PROGRESS LIMIT: "+string(ini_read_real("wavemodding","mod_boss_spawnvalue",11))
menuText[23] = "BOSSWAVE (NO END): "+string(ini_read_real("wavemodding","mod_bosswave",0))
menuText[24] = "ENDING ENABLED: "+string(ini_read_real("wavemodding","mod_ending",0))
menuText[25] = "MAKUTA HEALTH: "+string(ini_read_real("wavemodding","mod_health_makuta",3500))
menuText[26] = "PAUSE TIMER ON REVIVE: "+string(ini_read_real("wavemodding","mod_revive_timerpause",0))
menuText[27] = "REVIVAL LEVEL: "+string(ini_read_real("wavemodding","mod_revive_revivallevel",35))
menuText[28] = "RESPAWN TIME (BOT): "+string(ini_read_real("wavemodding","mod_respawntime_bot",150))
menuText[29] = "RESPAWN TIME (PLAYER): "+string(ini_read_real("wavemodding","mod_respawntime_player",150))
menuText[30] = "USE DEFUALT VALUES"
menuText[31] = "BACK TO MAIN MENU"
ini_close()
