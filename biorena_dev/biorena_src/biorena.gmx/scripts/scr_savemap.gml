levelname = get_save_filename('Game Map Files|*.gamemap','Map')
if file_exists(levelname) file_delete(levelname)
game_save(levelname);
