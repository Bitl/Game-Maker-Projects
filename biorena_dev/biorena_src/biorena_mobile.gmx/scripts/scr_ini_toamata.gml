ini_open("biorena_settings.ini")
name = ini_read_string("settings_player","toamata","Tahu")
global.toamata = name
ini_close()
