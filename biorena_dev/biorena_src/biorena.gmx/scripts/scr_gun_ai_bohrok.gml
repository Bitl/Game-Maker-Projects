//Argument 0: How many bullets are fired at once?
//Argument 1: What kind of bullet will be fired?
//Argument 2: What sound are we going to play?
//Argument 3: What fire rate are we going to fire at?
if (canfire == true)
{
    repeat(argument0)
    {
        instance_create(x,y,argument1)
        var bullet;
        bullet = argument1
    }
    sound_play(argument2)
    alarm[0]=argument3
    bullet.dmg = argument4
    energy-=1
    canfire = false
}
