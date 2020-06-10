//Shows a message on screen, and makes menu inactive as well as set value of variable in argument2 to last keyboard key.
sm_string = string(argument0);
sm_title = string(argument1);
sm_active = true;
sm_keygrab = true;
sm_keygrab_var = argument2;
sound_play(menu_snd_message);
