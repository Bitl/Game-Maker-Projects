#define settings_load
//Loads launchers settings into a ds_map

globalvar settings;
var i;

settings = ds_map_create();

ini_open("launcher.ini");
ds_map_add(settings,"lastupdate",ini_read_string("settings","lastupdate",""));
ds_map_add(settings,"host",ini_read_string("settings","host",""));
ds_map_add(settings,"filename",ini_read_string("settings","filename",""));
ds_map_add(settings,"executable",ini_read_string("settings","executable",""));

ds_map_add(settings,"launch_button",ini_read_string("buttons","launch",""));
ds_map_add(settings,"abort_button",ini_read_string("buttons","abort",""));
ds_map_add(settings,"retry_button",ini_read_string("buttons","retry",""));
ds_map_add(settings,"noupdate_button",ini_read_string("buttons","noupdate",""));
ds_map_add(settings,"update_button",ini_read_string("buttons","update",""));

ds_map_add(settings,"title",ini_read_string("text","title","Undefined"));
for(i=0;i<9;i+=1) {
    info[i] = ini_read_string("text","info"+string(i),"Undefined");
}

ini_close();


