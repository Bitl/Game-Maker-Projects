//First of all lets check if gui width or height is not equal to window width or height , if not then let's make them equal
if ( display_get_gui_width() != window_get_width() or display_get_gui_height() != window_get_height() ){
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
     * Finally we set window size and position back to what it was before then we make view size equal to window size, and it's done!
    */
    window_set_rectangle( xx , yy , ww , hh );
    surface_resize( application_surface , ww , hh )
    display_set_gui_size( ww , hh );
}
