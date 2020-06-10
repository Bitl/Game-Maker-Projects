//Adds a button to the menu.
// Arg0 = String
// Arg1 = Action (Code in string - executed when button is pressed)
// Arg2 = Type of button (0 = normal 1 = scrolling (ie-Something happens when you press left or right on it))
//(If you dont put an argument, normal type is used)
// - Arg3, 4 and 5 only needs arguments if type (Arg2) = 1.
// Arg3 = Action for left pressed (Code in string)
// Arg4 = Action for right pressed (Code in string)
// Arg5 = Variable attached (as a string) (This variable is added to the string of the variable - for when the button controls a variable)
//NOTE: Arg 5 can only be a local variable.
b_place += 1;
var i, n;
i = b_menu_append;
n = b_place;
//n = Button id, i = Menu id
b_effect[n,i] = 0;
b_str[n,i] = argument0;
b_action[n,i] = argument1;
b_num[i] += 1;
b_inputvar[n,i] = "";
b_separator[n,i] = false;
b_active[n,i] = true;
if (argument2 = 1)
{
    b_action_l[n,i] = argument3;
    b_action_r[n,i] = argument4;
    b_effect_l[n,i] = 0;
    b_effect_r[n,i] = 0;
}
else
{
    b_action_l[n,i] = "";
    b_action_r[n,i] = "";
}
if (string(argument5)="0") { b_variable[n,i] = "" } else { b_variable[n,i] = argument5 }
