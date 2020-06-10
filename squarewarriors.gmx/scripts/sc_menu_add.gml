//Adds a menu to keep buttons in/on.
//NOTE: You cant add buttons in any other event than the create event.
// Arg0 = x
// Arg1 = y
// Arg2 = Size (1 = normal size)
// Arg3 = Animated Background?
// Arg4 = Width (Set to 0 to enable auto-sizing)
// Arg5 = Menu Title (String) (Leave string empty if no title is needed)
b_menu_append += 1;
var i;
i = b_menu_append;
b_x[i] = argument0;
b_y[i] = argument1;
m_scale[i] = argument2;
b_anim[i] = argument3;
menu_width[i] = argument4;
b_num[i] = 1;
m_num += 1;
b_place = 0;
menu_title[i] = string(argument5);
if (string(argument5) = "0")
{
    menu_title[i] = "";
}
return i;
