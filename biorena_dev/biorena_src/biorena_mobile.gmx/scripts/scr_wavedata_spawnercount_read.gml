ini_open("biorena_savedwavedata.ini")
inibohrok=ini_read_real("wavedata","bohrok",5)
iniinfected=ini_read_real("wavedata","infected",4)
inirahkshi=ini_read_real("wavedata","rahkshi",2)
inimakuta=ini_read_real("wavedata","makuta",1)
inibohrok_spawner=ini_read_real("wavedata","bohrok_spawner",5)
iniinfected_spawner=ini_read_real("wavedata","infected_spawner",4)
inirahkshi_spawner=ini_read_real("wavedata","rahkshi_spawner",2)
inimakuta_spawner=ini_read_real("wavedata","makuta_spawner",1)
global.bohrak_count = inibohrok
global.infected_count = iniinfected
global.rahkshi_count = inirahkshi
global.makuta_count = inimakuta
global.spawner_bohrak_count = inibohrok_spawner
global.spawner_infected_count = iniinfected_spawner
global.spawner_rahkshi_count = inirahkshi_spawner
global.spawner_makuta_count = inimakuta_spawner
ini_close()
