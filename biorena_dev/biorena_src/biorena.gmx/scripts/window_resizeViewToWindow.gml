///window_resizeViewToWindow(view)
//First of all lets check if view width or height is not equal to window width or height , if not then let's make them equal
if ( view_wview[argument0] != window_get_width() or view_hview[argument0] != window_get_height() ){
    /*
     * We need to save window position and size for later usage
     * Because using display_reset() window size and position will be reset to what it was before.
    */
    var ww = window_get_width();
    var hh = window_get_height();
    var xx = window_get_x();
    var yy = window_get_y();
    /*
     * display_reset() is the key for pixel perfect resizing
     * Now we reset the display , without that it will look blurry and awful.
    */
    display_reset( 0 , false );
    /*
     * Finally we set window size and position back to what it was before then we make view sizes and display_gui sizes equal to window sizes, and it's done!
    */
    window_set_rectangle( xx , yy , ww , hh );
    surface_resize( application_surface , ww , hh )
    view_wview[argument0] = ww;
    view_hview[argument0] = hh;
    view_wport[argument0] = ww;
    view_hport[argument0] = hh;
}
