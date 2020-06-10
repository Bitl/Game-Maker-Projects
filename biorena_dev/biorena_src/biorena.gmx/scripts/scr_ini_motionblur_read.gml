ini_open("biorena_settings.ini")
iniblur = ini_read_real("settings_game","motionblur",1)
global.graphics_blurenable = iniblur
ini_close()
