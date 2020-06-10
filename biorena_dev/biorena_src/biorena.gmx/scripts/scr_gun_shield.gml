//Argument 0: What kind of bullet will be fired?
//Argument 1: What fire rate are we going to fire at?
if (mouse_check_button_released(mb_right))
{
    global.canfire_shield = true
    exit
}

if (mouse_check_button(mb_right) && global.canfire_shield == true && !global.shield_ammo < 1)
{
    instance_create(mouse_x,mouse_y,argument0)
    alarm[0]=argument1
    global.canfire_shield = false
}
