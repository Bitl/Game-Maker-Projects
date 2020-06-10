ini_open("biorena_settings.ini")
aimmode = ini_read_real("settings_controls","autoaim",1)
global.autoaim = aimmode
ini_close()
