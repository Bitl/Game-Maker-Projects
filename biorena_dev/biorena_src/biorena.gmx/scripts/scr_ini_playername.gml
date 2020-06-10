ini_open("biorena_settings.ini")
name = ini_read_string("settings_player","playername","Player")
global.playername = name
ini_close()
