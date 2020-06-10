//This script is just for setting up the menu. It is called in the create event of o_master.
//Main Menu //
menu_main = sc_menu_add(room_width/2,300,1.5,true,0,"Main Menu");
sc_menu_add_button("Start","MenuGoto(menu_info)");
sc_menu_add_button("Quit","game_end()");
//Info Menu //
menu_info = sc_menu_add(room_width/2,300,1.5,true,0,"Info Menu");
sc_menu_add_button("Show Info","sc_show_message('Hey!#This is some info about the menu. These menus can be edited in a matter of minutes - as long as you understand how it works! Just make sure you read the tutorial, which can be found in the GMC topic. =D','Information')");
sc_menu_add_button("Back","MenuGoto(menu_main)");
