if (mouse_check_button_released(mb_left))
{
    global.canfire = true
    exit
}

if (mouse_check_button(mb_left) && global.canfire == true)
{
    instance_create(x,y,obj_bullet)
    sound_play(snd_minigun)
    alarm[0]=global.minigun_rate
    global.canfire = false
}
