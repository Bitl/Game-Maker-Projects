var base_w = 640;
var base_h = 480;
var aspect = display_get_width() / display_get_height();
if (aspect > 1)
    {
    //landscape
    display_set_gui_size(base_h * aspect, base_h);
    }
else
    {
    //portrait
    display_set_gui_size(base_w, base_w / aspect);
    }
