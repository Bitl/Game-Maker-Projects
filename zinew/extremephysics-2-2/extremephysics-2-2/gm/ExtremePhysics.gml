#define extremephysics_init
var dllfile;
if is_string(argument0) then dllfile = argument0;
else dllfile = 'ExtremePhysics.dll';

global.define_ep_version = external_define(dllfile,'ep_version',dll_cdecl,ty_string,0);
global.define_ep_set_log_file = external_define(dllfile,'ep_set_log_file',dll_cdecl,ty_real,3,ty_real,ty_string,ty_real);
global.define_ep_set_show_errors = external_define(dllfile,'ep_set_show_errors',dll_cdecl,ty_real,1,ty_real);
global.define_ep_message = external_define(dllfile,'ep_message',dll_cdecl,ty_real,2,ty_real,ty_string);
global.define_ep_print_object_tree = external_define(dllfile,'ep_print_object_tree',dll_cdecl,ty_real,0);
global.define_ep_collision_test_box_box = external_define(dllfile,'ep_collision_test_box_box',dll_cdecl,ty_real,11,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_collision_test_box_line = external_define(dllfile,'ep_collision_test_box_line',dll_cdecl,ty_real,10,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_collision_test_box_circle = external_define(dllfile,'ep_collision_test_box_circle',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_collision_test_box_polygon = external_define(dllfile,'ep_collision_test_box_polygon',dll_cdecl,ty_real,11,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_collision_test_line_line = external_define(dllfile,'ep_collision_test_line_line',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_collision_test_line_circle = external_define(dllfile,'ep_collision_test_line_circle',dll_cdecl,ty_real,8,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_collision_test_line_polygon = external_define(dllfile,'ep_collision_test_line_polygon',dll_cdecl,ty_real,10,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_collision_test_circle_circle = external_define(dllfile,'ep_collision_test_circle_circle',dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_collision_test_circle_polygon = external_define(dllfile,'ep_collision_test_circle_polygon',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_collision_test_polygon_polygon = external_define(dllfile,'ep_collision_test_polygon_polygon',dll_cdecl,ty_real,11,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_ray_cast_box = external_define(dllfile,'ep_ray_cast_box',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_ray_cast_line = external_define(dllfile,'ep_ray_cast_line',dll_cdecl,ty_real,8,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_ray_cast_circle = external_define(dllfile,'ep_ray_cast_circle',dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_ray_cast_polygon = external_define(dllfile,'ep_ray_cast_polygon',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_first_world = external_define(dllfile,'ep_first_world',dll_cdecl,ty_real,0);
global.define_ep_last_world = external_define(dllfile,'ep_last_world',dll_cdecl,ty_real,0);
global.define_ep_gm_functions_init = external_define(dllfile,'ep_gm_functions_init',dll_cdecl,ty_real,0);
global.define_ep_gm_functions_init_address = external_define(dllfile,'ep_gm_functions_init_address',dll_cdecl,ty_real,1,ty_real);
global.define_ep_debugdraw_set_transformation = external_define(dllfile,'ep_debugdraw_set_transformation',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_debugdraw_bodies = external_define(dllfile,'ep_debugdraw_bodies',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_debugdraw_links = external_define(dllfile,'ep_debugdraw_links',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_debugdraw_views = external_define(dllfile,'ep_debugdraw_views',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_debugdraw_velocity = external_define(dllfile,'ep_debugdraw_velocity',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_debugdraw_forces = external_define(dllfile,'ep_debugdraw_forces',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_debugdraw_constraints = external_define(dllfile,'ep_debugdraw_constraints',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_world_create = external_define(dllfile,'ep_world_create',dll_cdecl,ty_real,0);
global.define_ep_world_destroy = external_define(dllfile,'ep_world_destroy',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_exists = external_define(dllfile,'ep_world_exists',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_clear = external_define(dllfile,'ep_world_clear',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_set_settings = external_define(dllfile,'ep_world_set_settings',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_world_set_primary_axis = external_define(dllfile,'ep_world_set_primary_axis',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_world_set_sleeping = external_define(dllfile,'ep_world_set_sleeping',dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_world_update_contacts = external_define(dllfile,'ep_world_update_contacts',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_simulate_step = external_define(dllfile,'ep_world_simulate_step',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_collision_test_box = external_define(dllfile,'ep_world_collision_test_box',dll_cdecl,ty_real,10,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_world_collision_test_line = external_define(dllfile,'ep_world_collision_test_line',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_world_collision_test_circle = external_define(dllfile,'ep_world_collision_test_circle',dll_cdecl,ty_real,8,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_world_collision_test_polygon = external_define(dllfile,'ep_world_collision_test_polygon',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_world_ray_cast = external_define(dllfile,'ep_world_ray_cast',dll_cdecl,ty_real,8,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_world_get_collision_body = external_define(dllfile,'ep_world_get_collision_body',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_world_get_collision_shape = external_define(dllfile,'ep_world_get_collision_shape',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_world_multipoly_begin = external_define(dllfile,'ep_world_multipoly_begin',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_world_multipoly_end = external_define(dllfile,'ep_world_multipoly_end',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_world_multipoly_set_vertex = external_define(dllfile,'ep_world_multipoly_set_vertex',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_world_multipoly_get_first = external_define(dllfile,'ep_world_multipoly_get_first',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_multipoly_get_last = external_define(dllfile,'ep_world_multipoly_get_last',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_serialize = external_define(dllfile,'ep_world_serialize',dll_cdecl,ty_string,1,ty_real);
global.define_ep_world_unserialize = external_define(dllfile,'ep_world_unserialize',dll_cdecl,ty_real,2,ty_real,ty_string);
global.define_ep_world_previous = external_define(dllfile,'ep_world_previous',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_next = external_define(dllfile,'ep_world_next',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_set_uservar = external_define(dllfile,'ep_world_set_uservar',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_world_get_uservar = external_define(dllfile,'ep_world_get_uservar',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_world_first_polygon = external_define(dllfile,'ep_world_first_polygon',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_last_polygon = external_define(dllfile,'ep_world_last_polygon',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_polygon_count = external_define(dllfile,'ep_world_polygon_count',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_first_body = external_define(dllfile,'ep_world_first_body',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_last_body = external_define(dllfile,'ep_world_last_body',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_body_count = external_define(dllfile,'ep_world_body_count',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_first_contact = external_define(dllfile,'ep_world_first_contact',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_last_contact = external_define(dllfile,'ep_world_last_contact',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_contact_count = external_define(dllfile,'ep_world_contact_count',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_first_hingejoint = external_define(dllfile,'ep_world_first_hingejoint',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_last_hingejoint = external_define(dllfile,'ep_world_last_hingejoint',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_hingejoint_count = external_define(dllfile,'ep_world_hingejoint_count',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_first_distancejoint = external_define(dllfile,'ep_world_first_distancejoint',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_last_distancejoint = external_define(dllfile,'ep_world_last_distancejoint',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_distancejoint_count = external_define(dllfile,'ep_world_distancejoint_count',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_first_railjoint = external_define(dllfile,'ep_world_first_railjoint',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_last_railjoint = external_define(dllfile,'ep_world_last_railjoint',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_railjoint_count = external_define(dllfile,'ep_world_railjoint_count',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_first_sliderjoint = external_define(dllfile,'ep_world_first_sliderjoint',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_last_sliderjoint = external_define(dllfile,'ep_world_last_sliderjoint',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_sliderjoint_count = external_define(dllfile,'ep_world_sliderjoint_count',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_first_view = external_define(dllfile,'ep_world_first_view',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_last_view = external_define(dllfile,'ep_world_last_view',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_view_count = external_define(dllfile,'ep_world_view_count',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_first_water = external_define(dllfile,'ep_world_first_water',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_last_water = external_define(dllfile,'ep_world_last_water',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_water_count = external_define(dllfile,'ep_world_water_count',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_shape_count = external_define(dllfile,'ep_world_shape_count',dll_cdecl,ty_real,1,ty_real);
global.define_ep_world_force_count = external_define(dllfile,'ep_world_force_count',dll_cdecl,ty_real,1,ty_real);
global.define_ep_polygon_create = external_define(dllfile,'ep_polygon_create',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_polygon_destroy = external_define(dllfile,'ep_polygon_destroy',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_polygon_exists = external_define(dllfile,'ep_polygon_exists',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_polygon_set_vertex = external_define(dllfile,'ep_polygon_set_vertex',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_polygon_initialize = external_define(dllfile,'ep_polygon_initialize',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_polygon_get_vertex_count = external_define(dllfile,'ep_polygon_get_vertex_count',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_polygon_get_vertex_x = external_define(dllfile,'ep_polygon_get_vertex_x',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_polygon_get_vertex_y = external_define(dllfile,'ep_polygon_get_vertex_y',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_polygon_get_vertex_normal_x = external_define(dllfile,'ep_polygon_get_vertex_normal_x',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_polygon_get_vertex_normal_y = external_define(dllfile,'ep_polygon_get_vertex_normal_y',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_polygon_get_edge_length = external_define(dllfile,'ep_polygon_get_edge_length',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_polygon_previous = external_define(dllfile,'ep_polygon_previous',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_polygon_next = external_define(dllfile,'ep_polygon_next',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_polygon_set_uservar = external_define(dllfile,'ep_polygon_set_uservar',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_polygon_get_uservar = external_define(dllfile,'ep_polygon_get_uservar',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_body_create_static = external_define(dllfile,'ep_body_create_static',dll_cdecl,ty_real,1,ty_real);
global.define_ep_body_create_dynamic = external_define(dllfile,'ep_body_create_dynamic',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_destroy = external_define(dllfile,'ep_body_destroy',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_exists = external_define(dllfile,'ep_body_exists',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_first_hingejoint = external_define(dllfile,'ep_body_get_first_hingejoint',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_last_hingejoint = external_define(dllfile,'ep_body_get_last_hingejoint',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_previous_hingejoint = external_define(dllfile,'ep_body_get_previous_hingejoint',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_body_get_next_hingejoint = external_define(dllfile,'ep_body_get_next_hingejoint',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_body_get_first_distancejoint = external_define(dllfile,'ep_body_get_first_distancejoint',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_last_distancejoint = external_define(dllfile,'ep_body_get_last_distancejoint',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_previous_distancejoint = external_define(dllfile,'ep_body_get_previous_distancejoint',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_body_get_next_distancejoint = external_define(dllfile,'ep_body_get_next_distancejoint',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_body_get_first_railjoint = external_define(dllfile,'ep_body_get_first_railjoint',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_last_railjoint = external_define(dllfile,'ep_body_get_last_railjoint',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_previous_railjoint = external_define(dllfile,'ep_body_get_previous_railjoint',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_body_get_next_railjoint = external_define(dllfile,'ep_body_get_next_railjoint',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_body_get_first_sliderjoint = external_define(dllfile,'ep_body_get_first_sliderjoint',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_last_sliderjoint = external_define(dllfile,'ep_body_get_last_sliderjoint',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_previous_sliderjoint = external_define(dllfile,'ep_body_get_previous_sliderjoint',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_body_get_next_sliderjoint = external_define(dllfile,'ep_body_get_next_sliderjoint',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_body_calculate_mass = external_define(dllfile,'ep_body_calculate_mass',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_set_mass = external_define(dllfile,'ep_body_set_mass',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_body_set_inertia = external_define(dllfile,'ep_body_set_inertia',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_body_set_center = external_define(dllfile,'ep_body_set_center',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_set_position = external_define(dllfile,'ep_body_set_position',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_set_position_center = external_define(dllfile,'ep_body_set_position_center',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_set_position_local_point = external_define(dllfile,'ep_body_set_position_local_point',dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_set_velocity_center = external_define(dllfile,'ep_body_set_velocity_center',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_set_velocity_local_point = external_define(dllfile,'ep_body_set_velocity_local_point',dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_set_max_velocity = external_define(dllfile,'ep_body_set_max_velocity',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_set_gravity = external_define(dllfile,'ep_body_set_gravity',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_set_damping = external_define(dllfile,'ep_body_set_damping',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_store_impulses = external_define(dllfile,'ep_body_store_impulses',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_set_sleeping = external_define(dllfile,'ep_body_set_sleeping',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_collision_test_box = external_define(dllfile,'ep_body_collision_test_box',dll_cdecl,ty_real,11,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_collision_test_line = external_define(dllfile,'ep_body_collision_test_line',dll_cdecl,ty_real,10,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_collision_test_circle = external_define(dllfile,'ep_body_collision_test_circle',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_collision_test_polygon = external_define(dllfile,'ep_body_collision_test_polygon',dll_cdecl,ty_real,10,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_ray_cast = external_define(dllfile,'ep_body_ray_cast',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_apply_impulse = external_define(dllfile,'ep_body_apply_impulse',dll_cdecl,ty_real,10,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_apply_impulse_relative = external_define(dllfile,'ep_body_apply_impulse_relative',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_get_mass = external_define(dllfile,'ep_body_get_mass',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_inertia = external_define(dllfile,'ep_body_get_inertia',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_center_of_mass_x = external_define(dllfile,'ep_body_get_center_of_mass_x',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_center_of_mass_y = external_define(dllfile,'ep_body_get_center_of_mass_y',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_x = external_define(dllfile,'ep_body_get_x',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_y = external_define(dllfile,'ep_body_get_y',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_x_center = external_define(dllfile,'ep_body_get_x_center',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_y_center = external_define(dllfile,'ep_body_get_y_center',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_rot = external_define(dllfile,'ep_body_get_rot',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_xvel_center = external_define(dllfile,'ep_body_get_xvel_center',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_yvel_center = external_define(dllfile,'ep_body_get_yvel_center',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_get_xvel_local_point = external_define(dllfile,'ep_body_get_xvel_local_point',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_get_yvel_local_point = external_define(dllfile,'ep_body_get_yvel_local_point',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_get_rotvel = external_define(dllfile,'ep_body_get_rotvel',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_is_static = external_define(dllfile,'ep_body_is_static',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_is_norotation = external_define(dllfile,'ep_body_is_norotation',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_is_sleeping = external_define(dllfile,'ep_body_is_sleeping',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_stable_timer = external_define(dllfile,'ep_body_stable_timer',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_out_of_view_timer = external_define(dllfile,'ep_body_out_of_view_timer',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_coord_local_to_world_x = external_define(dllfile,'ep_body_coord_local_to_world_x',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_coord_local_to_world_y = external_define(dllfile,'ep_body_coord_local_to_world_y',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_coord_world_to_local_x = external_define(dllfile,'ep_body_coord_world_to_local_x',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_coord_world_to_local_y = external_define(dllfile,'ep_body_coord_world_to_local_y',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_vect_local_to_world_x = external_define(dllfile,'ep_body_vect_local_to_world_x',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_vect_local_to_world_y = external_define(dllfile,'ep_body_vect_local_to_world_y',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_vect_world_to_local_x = external_define(dllfile,'ep_body_vect_world_to_local_x',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_vect_world_to_local_y = external_define(dllfile,'ep_body_vect_world_to_local_y',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_boxchain_begin = external_define(dllfile,'ep_body_boxchain_begin',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_body_boxchain_end = external_define(dllfile,'ep_body_boxchain_end',dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_boxchain_set_vertex = external_define(dllfile,'ep_body_boxchain_set_vertex',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_boxchain_get_first = external_define(dllfile,'ep_body_boxchain_get_first',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_boxchain_get_last = external_define(dllfile,'ep_body_boxchain_get_last',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_previous = external_define(dllfile,'ep_body_previous',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_next = external_define(dllfile,'ep_body_next',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_set_uservar = external_define(dllfile,'ep_body_set_uservar',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_body_get_uservar = external_define(dllfile,'ep_body_get_uservar',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_body_first_shape = external_define(dllfile,'ep_body_first_shape',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_last_shape = external_define(dllfile,'ep_body_last_shape',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_shape_count = external_define(dllfile,'ep_body_shape_count',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_first_force = external_define(dllfile,'ep_body_first_force',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_last_force = external_define(dllfile,'ep_body_last_force',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_body_force_count = external_define(dllfile,'ep_body_force_count',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_destroy = external_define(dllfile,'ep_contact_destroy',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_exists = external_define(dllfile,'ep_contact_exists',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_body1 = external_define(dllfile,'ep_contact_get_body1',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_body2 = external_define(dllfile,'ep_contact_get_body2',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_shape1 = external_define(dllfile,'ep_contact_get_shape1',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_shape2 = external_define(dllfile,'ep_contact_get_shape2',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_normal_x = external_define(dllfile,'ep_contact_get_normal_x',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_normal_y = external_define(dllfile,'ep_contact_get_normal_y',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point1_active = external_define(dllfile,'ep_contact_get_point1_active',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point2_active = external_define(dllfile,'ep_contact_get_point2_active',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point1_x = external_define(dllfile,'ep_contact_get_point1_x',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point1_y = external_define(dllfile,'ep_contact_get_point1_y',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point2_x = external_define(dllfile,'ep_contact_get_point2_x',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point2_y = external_define(dllfile,'ep_contact_get_point2_y',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point1_separation = external_define(dllfile,'ep_contact_get_point1_separation',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point2_separation = external_define(dllfile,'ep_contact_get_point2_separation',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point1_normalforce = external_define(dllfile,'ep_contact_get_point1_normalforce',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point1_tangentforce = external_define(dllfile,'ep_contact_get_point1_tangentforce',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point2_normalforce = external_define(dllfile,'ep_contact_get_point2_normalforce',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point2_tangentforce = external_define(dllfile,'ep_contact_get_point2_tangentforce',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point1_normalveldelta = external_define(dllfile,'ep_contact_get_point1_normalveldelta',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point1_tangentveldelta = external_define(dllfile,'ep_contact_get_point1_tangentveldelta',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point2_normalveldelta = external_define(dllfile,'ep_contact_get_point2_normalveldelta',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_get_point2_tangentveldelta = external_define(dllfile,'ep_contact_get_point2_tangentveldelta',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_previous = external_define(dllfile,'ep_contact_previous',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_contact_next = external_define(dllfile,'ep_contact_next',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_hingejoint_create = external_define(dllfile,'ep_hingejoint_create',dll_cdecl,ty_real,8,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_hingejoint_destroy = external_define(dllfile,'ep_hingejoint_destroy',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_hingejoint_exists = external_define(dllfile,'ep_hingejoint_exists',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_hingejoint_set_max_force = external_define(dllfile,'ep_hingejoint_set_max_force',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_hingejoint_set_motor = external_define(dllfile,'ep_hingejoint_set_motor',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_hingejoint_set_limit_settings = external_define(dllfile,'ep_hingejoint_set_limit_settings',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_hingejoint_set_lower_limit = external_define(dllfile,'ep_hingejoint_set_lower_limit',dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_hingejoint_set_upper_limit = external_define(dllfile,'ep_hingejoint_set_upper_limit',dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_hingejoint_set_lower_spring = external_define(dllfile,'ep_hingejoint_set_lower_spring',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_hingejoint_set_upper_spring = external_define(dllfile,'ep_hingejoint_set_upper_spring',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_hingejoint_get_body1 = external_define(dllfile,'ep_hingejoint_get_body1',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_hingejoint_get_body2 = external_define(dllfile,'ep_hingejoint_get_body2',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_hingejoint_get_rotation = external_define(dllfile,'ep_hingejoint_get_rotation',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_hingejoint_get_xforce = external_define(dllfile,'ep_hingejoint_get_xforce',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_hingejoint_get_yforce = external_define(dllfile,'ep_hingejoint_get_yforce',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_hingejoint_get_motor_torque = external_define(dllfile,'ep_hingejoint_get_motor_torque',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_hingejoint_get_limit_torque = external_define(dllfile,'ep_hingejoint_get_limit_torque',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_hingejoint_previous = external_define(dllfile,'ep_hingejoint_previous',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_hingejoint_next = external_define(dllfile,'ep_hingejoint_next',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_hingejoint_set_uservar = external_define(dllfile,'ep_hingejoint_set_uservar',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_hingejoint_get_uservar = external_define(dllfile,'ep_hingejoint_get_uservar',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_distancejoint_create = external_define(dllfile,'ep_distancejoint_create',dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_distancejoint_destroy = external_define(dllfile,'ep_distancejoint_destroy',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_distancejoint_exists = external_define(dllfile,'ep_distancejoint_exists',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_distancejoint_set_motor = external_define(dllfile,'ep_distancejoint_set_motor',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_distancejoint_set_limit_settings = external_define(dllfile,'ep_distancejoint_set_limit_settings',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_distancejoint_set_lower_limit = external_define(dllfile,'ep_distancejoint_set_lower_limit',dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_distancejoint_set_upper_limit = external_define(dllfile,'ep_distancejoint_set_upper_limit',dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_distancejoint_set_lower_spring = external_define(dllfile,'ep_distancejoint_set_lower_spring',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_distancejoint_set_upper_spring = external_define(dllfile,'ep_distancejoint_set_upper_spring',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_distancejoint_get_body1 = external_define(dllfile,'ep_distancejoint_get_body1',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_distancejoint_get_body2 = external_define(dllfile,'ep_distancejoint_get_body2',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_distancejoint_get_distance = external_define(dllfile,'ep_distancejoint_get_distance',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_distancejoint_get_motor_force = external_define(dllfile,'ep_distancejoint_get_motor_force',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_distancejoint_get_limit_force = external_define(dllfile,'ep_distancejoint_get_limit_force',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_distancejoint_previous = external_define(dllfile,'ep_distancejoint_previous',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_distancejoint_next = external_define(dllfile,'ep_distancejoint_next',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_distancejoint_set_uservar = external_define(dllfile,'ep_distancejoint_set_uservar',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_distancejoint_get_uservar = external_define(dllfile,'ep_distancejoint_get_uservar',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_railjoint_create = external_define(dllfile,'ep_railjoint_create',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_railjoint_destroy = external_define(dllfile,'ep_railjoint_destroy',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_railjoint_exists = external_define(dllfile,'ep_railjoint_exists',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_railjoint_set_max_normal_force = external_define(dllfile,'ep_railjoint_set_max_normal_force',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_railjoint_set_motor = external_define(dllfile,'ep_railjoint_set_motor',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_railjoint_set_limit_settings = external_define(dllfile,'ep_railjoint_set_limit_settings',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_railjoint_set_lower_limit = external_define(dllfile,'ep_railjoint_set_lower_limit',dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_railjoint_set_upper_limit = external_define(dllfile,'ep_railjoint_set_upper_limit',dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_railjoint_set_lower_spring = external_define(dllfile,'ep_railjoint_set_lower_spring',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_railjoint_set_upper_spring = external_define(dllfile,'ep_railjoint_set_upper_spring',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_railjoint_get_body1 = external_define(dllfile,'ep_railjoint_get_body1',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_railjoint_get_body2 = external_define(dllfile,'ep_railjoint_get_body2',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_railjoint_get_position = external_define(dllfile,'ep_railjoint_get_position',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_railjoint_get_normal_force = external_define(dllfile,'ep_railjoint_get_normal_force',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_railjoint_get_motor_force = external_define(dllfile,'ep_railjoint_get_motor_force',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_railjoint_get_limit_force = external_define(dllfile,'ep_railjoint_get_limit_force',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_railjoint_previous = external_define(dllfile,'ep_railjoint_previous',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_railjoint_next = external_define(dllfile,'ep_railjoint_next',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_railjoint_set_uservar = external_define(dllfile,'ep_railjoint_set_uservar',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_railjoint_get_uservar = external_define(dllfile,'ep_railjoint_get_uservar',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_sliderjoint_create = external_define(dllfile,'ep_sliderjoint_create',dll_cdecl,ty_real,10,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_sliderjoint_destroy = external_define(dllfile,'ep_sliderjoint_destroy',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_sliderjoint_exists = external_define(dllfile,'ep_sliderjoint_exists',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_sliderjoint_set_max_combined_force = external_define(dllfile,'ep_sliderjoint_set_max_combined_force',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_sliderjoint_set_motor = external_define(dllfile,'ep_sliderjoint_set_motor',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_sliderjoint_set_limit_settings = external_define(dllfile,'ep_sliderjoint_set_limit_settings',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_sliderjoint_set_lower_limit = external_define(dllfile,'ep_sliderjoint_set_lower_limit',dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_sliderjoint_set_upper_limit = external_define(dllfile,'ep_sliderjoint_set_upper_limit',dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_sliderjoint_set_lower_spring = external_define(dllfile,'ep_sliderjoint_set_lower_spring',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_sliderjoint_set_upper_spring = external_define(dllfile,'ep_sliderjoint_set_upper_spring',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_sliderjoint_get_body1 = external_define(dllfile,'ep_sliderjoint_get_body1',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_sliderjoint_get_body2 = external_define(dllfile,'ep_sliderjoint_get_body2',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_sliderjoint_get_position = external_define(dllfile,'ep_sliderjoint_get_position',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_sliderjoint_get_normal_force = external_define(dllfile,'ep_sliderjoint_get_normal_force',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_sliderjoint_get_torque = external_define(dllfile,'ep_sliderjoint_get_torque',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_sliderjoint_get_combined_force = external_define(dllfile,'ep_sliderjoint_get_combined_force',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_sliderjoint_get_motor_force = external_define(dllfile,'ep_sliderjoint_get_motor_force',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_sliderjoint_get_limit_force = external_define(dllfile,'ep_sliderjoint_get_limit_force',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_sliderjoint_previous = external_define(dllfile,'ep_sliderjoint_previous',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_sliderjoint_next = external_define(dllfile,'ep_sliderjoint_next',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_sliderjoint_set_uservar = external_define(dllfile,'ep_sliderjoint_set_uservar',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_sliderjoint_get_uservar = external_define(dllfile,'ep_sliderjoint_get_uservar',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_view_create = external_define(dllfile,'ep_view_create',dll_cdecl,ty_real,1,ty_real);
global.define_ep_view_destroy = external_define(dllfile,'ep_view_destroy',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_view_exists = external_define(dllfile,'ep_view_exists',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_view_set_rectangle = external_define(dllfile,'ep_view_set_rectangle',dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_view_previous = external_define(dllfile,'ep_view_previous',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_view_next = external_define(dllfile,'ep_view_next',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_view_set_uservar = external_define(dllfile,'ep_view_set_uservar',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_view_get_uservar = external_define(dllfile,'ep_view_get_uservar',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_water_create = external_define(dllfile,'ep_water_create',dll_cdecl,ty_real,1,ty_real);
global.define_ep_water_destroy = external_define(dllfile,'ep_water_destroy',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_water_exists = external_define(dllfile,'ep_water_exists',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_water_set_parameters = external_define(dllfile,'ep_water_set_parameters',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_water_set_rectangle = external_define(dllfile,'ep_water_set_rectangle',dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_water_previous = external_define(dllfile,'ep_water_previous',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_water_next = external_define(dllfile,'ep_water_next',dll_cdecl,ty_real,2,ty_real,ty_real);
global.define_ep_water_set_uservar = external_define(dllfile,'ep_water_set_uservar',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_water_get_uservar = external_define(dllfile,'ep_water_get_uservar',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_shape_create_box = external_define(dllfile,'ep_shape_create_box',dll_cdecl,ty_real,8,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_create_line = external_define(dllfile,'ep_shape_create_line',dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_create_circle = external_define(dllfile,'ep_shape_create_circle',dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_create_polygon = external_define(dllfile,'ep_shape_create_polygon',dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_destroy = external_define(dllfile,'ep_shape_destroy',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_shape_exists = external_define(dllfile,'ep_shape_exists',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_shape_get_first_contact = external_define(dllfile,'ep_shape_get_first_contact',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_shape_get_last_contact = external_define(dllfile,'ep_shape_get_last_contact',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_shape_get_previous_contact = external_define(dllfile,'ep_shape_get_previous_contact',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_get_next_contact = external_define(dllfile,'ep_shape_get_next_contact',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_set_material = external_define(dllfile,'ep_shape_set_material',dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_set_collision = external_define(dllfile,'ep_shape_set_collision',dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_collision_test_box = external_define(dllfile,'ep_shape_collision_test_box',dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_collision_test_line = external_define(dllfile,'ep_shape_collision_test_line',dll_cdecl,ty_real,8,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_collision_test_circle = external_define(dllfile,'ep_shape_collision_test_circle',dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_collision_test_polygon = external_define(dllfile,'ep_shape_collision_test_polygon',dll_cdecl,ty_real,8,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_ray_cast = external_define(dllfile,'ep_shape_ray_cast',dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_previous = external_define(dllfile,'ep_shape_previous',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_shape_next = external_define(dllfile,'ep_shape_next',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_shape_set_uservar = external_define(dllfile,'ep_shape_set_uservar',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_shape_get_uservar = external_define(dllfile,'ep_shape_get_uservar',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.define_ep_force_create = external_define(dllfile,'ep_force_create',dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_force_destroy = external_define(dllfile,'ep_force_destroy',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_force_exists = external_define(dllfile,'ep_force_exists',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_force_set_force = external_define(dllfile,'ep_force_set_force',dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_force_previous = external_define(dllfile,'ep_force_previous',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_force_next = external_define(dllfile,'ep_force_next',dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.define_ep_force_set_uservar = external_define(dllfile,'ep_force_set_uservar',dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.define_ep_force_get_uservar = external_define(dllfile,'ep_force_get_uservar',dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);

#define ep_version
// ep_version()
external_call(global.define_ep_version);
#define ep_set_log_file
// ep_set_log_file(enable,filename,level)
external_call(global.define_ep_set_log_file,argument0,argument1,argument2);
#define ep_set_show_errors
// ep_set_show_errors(enable)
external_call(global.define_ep_set_show_errors,argument0);
#define ep_message
// ep_message(level,string)
external_call(global.define_ep_message,argument0,argument1);
#define ep_print_object_tree
// ep_print_object_tree()
external_call(global.define_ep_print_object_tree);
#define ep_collision_test_box_box
// ep_collision_test_box_box(shape1_w,shape1_h,shape1_x,shape1_y,shape1_rot,shape2_w,shape2_h,shape2_x,shape2_y,shape2_rot,contact_threshold)
external_call(global.define_ep_collision_test_box_box,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8,argument9,argument10);
#define ep_collision_test_box_line
// ep_collision_test_box_line(shape1_w,shape1_h,shape1_x,shape1_y,shape1_rot,shape2_x1,shape2_y1,shape2_x2,shape2_y2,contact_threshold)
external_call(global.define_ep_collision_test_box_line,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8,argument9);
#define ep_collision_test_box_circle
// ep_collision_test_box_circle(shape1_w,shape1_h,shape1_x,shape1_y,shape1_rot,shape2_r,shape2_x,shape2_y,contact_threshold)
external_call(global.define_ep_collision_test_box_circle,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_collision_test_box_polygon
// ep_collision_test_box_polygon(shape1_w,shape1_h,shape1_x,shape1_y,shape1_rot,shape2_world_id,shape2_polygon_id,shape2_x,shape2_y,shape2_rot,contact_threshold)
external_call(global.define_ep_collision_test_box_polygon,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8,argument9,argument10);
#define ep_collision_test_line_line
// ep_collision_test_line_line(shape1_x1,shape1_y1,shape1_x2,shape1_y2,shape2_x1,shape2_y1,shape2_x2,shape2_y2,contact_threshold)
external_call(global.define_ep_collision_test_line_line,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_collision_test_line_circle
// ep_collision_test_line_circle(shape1_x1,shape1_y1,shape1_x2,shape1_y2,shape2_r,shape2_x,shape2_y,contact_threshold)
external_call(global.define_ep_collision_test_line_circle,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7);
#define ep_collision_test_line_polygon
// ep_collision_test_line_polygon(shape1_x1,shape1_y1,shape1_x2,shape1_y2,shape2_world_id,shape2_polygon_id,shape2_x,shape2_y,shape2_rot,contact_threshold)
external_call(global.define_ep_collision_test_line_polygon,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8,argument9);
#define ep_collision_test_circle_circle
// ep_collision_test_circle_circle(shape1_r,shape1_x,shape1_y,shape2_r,shape2_x,shape2_y,contact_threshold)
external_call(global.define_ep_collision_test_circle_circle,argument0,argument1,argument2,argument3,argument4,argument5,argument6);
#define ep_collision_test_circle_polygon
// ep_collision_test_circle_polygon(shape1_r,shape1_x,shape1_y,shape2_world_id,shape2_polygon_id,shape2_x,shape2_y,shape2_rot,contact_threshold)
external_call(global.define_ep_collision_test_circle_polygon,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_collision_test_polygon_polygon
// ep_collision_test_polygon_polygon(shape1_world_id,shape1_polygon_id,shape1_x,shape1_y,shape1_rot,shape2_world_id,shape2_polygon_id,shape2_x,shape2_y,shape2_rot,contact_threshold)
external_call(global.define_ep_collision_test_polygon_polygon,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8,argument9,argument10);
#define ep_ray_cast_box
// ep_ray_cast_box(ray_x,ray_y,ray_vx,ray_vy,shape_w,shape_h,shape_x,shape_y,shape_rot)
external_call(global.define_ep_ray_cast_box,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_ray_cast_line
// ep_ray_cast_line(ray_x,ray_y,ray_vx,ray_vy,shape_x1,shape_y1,shape_x2,shape_y2)
external_call(global.define_ep_ray_cast_line,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7);
#define ep_ray_cast_circle
// ep_ray_cast_circle(ray_x,ray_y,ray_vx,ray_vy,shape_r,shape_x,shape_y)
external_call(global.define_ep_ray_cast_circle,argument0,argument1,argument2,argument3,argument4,argument5,argument6);
#define ep_ray_cast_polygon
// ep_ray_cast_polygon(ray_x,ray_y,ray_vx,ray_vy,shape_world_id,shape_polygon_id,shape_x,shape_y,shape_rot)
external_call(global.define_ep_ray_cast_polygon,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_first_world
// ep_first_world()
external_call(global.define_ep_first_world);
#define ep_last_world
// ep_last_world()
external_call(global.define_ep_last_world);
#define ep_gm_functions_init
// ep_gm_functions_init()
external_call(global.define_ep_gm_functions_init);
#define ep_gm_functions_init_address
// ep_gm_functions_init_address(address)
external_call(global.define_ep_gm_functions_init_address,argument0);
#define ep_debugdraw_set_transformation
// ep_debugdraw_set_transformation(translation_x,translation_y,rotation,scale)
external_call(global.define_ep_debugdraw_set_transformation,argument0,argument1,argument2,argument3);
#define ep_debugdraw_bodies
// ep_debugdraw_bodies(world_id,color_static,color_dynamic)
external_call(global.define_ep_debugdraw_bodies,argument0,argument1,argument2);
#define ep_debugdraw_links
// ep_debugdraw_links(world_id,color_shapelink,color_contactlink,color_jointlink)
external_call(global.define_ep_debugdraw_links,argument0,argument1,argument2,argument3);
#define ep_debugdraw_views
// ep_debugdraw_views(world_id,color_view)
external_call(global.define_ep_debugdraw_views,argument0,argument1);
#define ep_debugdraw_velocity
// ep_debugdraw_velocity(world_id,color_velocity,scale,rotscale)
external_call(global.define_ep_debugdraw_velocity,argument0,argument1,argument2,argument3);
#define ep_debugdraw_forces
// ep_debugdraw_forces(world_id,color_force,scale,rotscale)
external_call(global.define_ep_debugdraw_forces,argument0,argument1,argument2,argument3);
#define ep_debugdraw_constraints
// ep_debugdraw_constraints(world_id,color_contactpoint,color_joint)
external_call(global.define_ep_debugdraw_constraints,argument0,argument1,argument2);
#define ep_world_create
// ep_world_create()
external_call(global.define_ep_world_create);
#define ep_world_destroy
// ep_world_destroy(world_id)
external_call(global.define_ep_world_destroy,argument0);
#define ep_world_exists
// ep_world_exists(world_id)
external_call(global.define_ep_world_exists,argument0);
#define ep_world_clear
// ep_world_clear(world_id)
external_call(global.define_ep_world_clear,argument0);
#define ep_world_set_settings
// ep_world_set_settings(world_id,timestep,velocity_iterations,position_iterations,contact_threshold,velocity_threshold,baumgarte_factor,mass_bias,position_factor)
external_call(global.define_ep_world_set_settings,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_world_set_primary_axis
// ep_world_set_primary_axis(world_id,horizontal)
external_call(global.define_ep_world_set_primary_axis,argument0,argument1);
#define ep_world_set_sleeping
// ep_world_set_sleeping(world_id,enable_sleeping,time_stable,time_outofview,stable_maxvel,stable_maxrotvel)
external_call(global.define_ep_world_set_sleeping,argument0,argument1,argument2,argument3,argument4,argument5);
#define ep_world_update_contacts
// ep_world_update_contacts(world_id)
external_call(global.define_ep_world_update_contacts,argument0);
#define ep_world_simulate_step
// ep_world_simulate_step(world_id)
external_call(global.define_ep_world_simulate_step,argument0);
#define ep_world_collision_test_box
// ep_world_collision_test_box(world_id,w,h,x,y,rot,contact_threshold,collidemask1,collidemask2,group)
external_call(global.define_ep_world_collision_test_box,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8,argument9);
#define ep_world_collision_test_line
// ep_world_collision_test_line(world_id,x1,y1,x2,y2,contact_threshold,collidemask1,collidemask2,group)
external_call(global.define_ep_world_collision_test_line,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_world_collision_test_circle
// ep_world_collision_test_circle(world_id,r,x,y,contact_threshold,collidemask1,collidemask2,group)
external_call(global.define_ep_world_collision_test_circle,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7);
#define ep_world_collision_test_polygon
// ep_world_collision_test_polygon(world_id,polygon_id,x,y,rot,contact_threshold,collidemask1,collidemask2,group)
external_call(global.define_ep_world_collision_test_polygon,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_world_ray_cast
// ep_world_ray_cast(world_id,x,y,vx,vy,collidemask1,collidemask2,group)
external_call(global.define_ep_world_ray_cast,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7);
#define ep_world_get_collision_body
// ep_world_get_collision_body(world_id,index)
external_call(global.define_ep_world_get_collision_body,argument0,argument1);
#define ep_world_get_collision_shape
// ep_world_get_collision_shape(world_id,index)
external_call(global.define_ep_world_get_collision_shape,argument0,argument1);
#define ep_world_multipoly_begin
// ep_world_multipoly_begin(world_id,vertexcount)
external_call(global.define_ep_world_multipoly_begin,argument0,argument1);
#define ep_world_multipoly_end
// ep_world_multipoly_end(world_id,showerrors)
external_call(global.define_ep_world_multipoly_end,argument0,argument1);
#define ep_world_multipoly_set_vertex
// ep_world_multipoly_set_vertex(world_id,index,x,y)
external_call(global.define_ep_world_multipoly_set_vertex,argument0,argument1,argument2,argument3);
#define ep_world_multipoly_get_first
// ep_world_multipoly_get_first(world_id)
external_call(global.define_ep_world_multipoly_get_first,argument0);
#define ep_world_multipoly_get_last
// ep_world_multipoly_get_last(world_id)
external_call(global.define_ep_world_multipoly_get_last,argument0);
#define ep_world_serialize
// ep_world_serialize(world_id)
external_call(global.define_ep_world_serialize,argument0);
#define ep_world_unserialize
// ep_world_unserialize(world_id,data)
external_call(global.define_ep_world_unserialize,argument0,argument1);
#define ep_world_previous
// ep_world_previous(world_id)
external_call(global.define_ep_world_previous,argument0);
#define ep_world_next
// ep_world_next(world_id)
external_call(global.define_ep_world_next,argument0);
#define ep_world_set_uservar
// ep_world_set_uservar(world_id,index,value)
external_call(global.define_ep_world_set_uservar,argument0,argument1,argument2);
#define ep_world_get_uservar
// ep_world_get_uservar(world_id,body_id,index)
external_call(global.define_ep_world_get_uservar,argument0,argument1,argument2);
#define ep_world_first_polygon
// ep_world_first_polygon(world_id)
external_call(global.define_ep_world_first_polygon,argument0);
#define ep_world_last_polygon
// ep_world_last_polygon(world_id)
external_call(global.define_ep_world_last_polygon,argument0);
#define ep_world_polygon_count
// ep_world_polygon_count(world_id)
external_call(global.define_ep_world_polygon_count,argument0);
#define ep_world_first_body
// ep_world_first_body(world_id)
external_call(global.define_ep_world_first_body,argument0);
#define ep_world_last_body
// ep_world_last_body(world_id)
external_call(global.define_ep_world_last_body,argument0);
#define ep_world_body_count
// ep_world_body_count(world_id)
external_call(global.define_ep_world_body_count,argument0);
#define ep_world_first_contact
// ep_world_first_contact(world_id)
external_call(global.define_ep_world_first_contact,argument0);
#define ep_world_last_contact
// ep_world_last_contact(world_id)
external_call(global.define_ep_world_last_contact,argument0);
#define ep_world_contact_count
// ep_world_contact_count(world_id)
external_call(global.define_ep_world_contact_count,argument0);
#define ep_world_first_hingejoint
// ep_world_first_hingejoint(world_id)
external_call(global.define_ep_world_first_hingejoint,argument0);
#define ep_world_last_hingejoint
// ep_world_last_hingejoint(world_id)
external_call(global.define_ep_world_last_hingejoint,argument0);
#define ep_world_hingejoint_count
// ep_world_hingejoint_count(world_id)
external_call(global.define_ep_world_hingejoint_count,argument0);
#define ep_world_first_distancejoint
// ep_world_first_distancejoint(world_id)
external_call(global.define_ep_world_first_distancejoint,argument0);
#define ep_world_last_distancejoint
// ep_world_last_distancejoint(world_id)
external_call(global.define_ep_world_last_distancejoint,argument0);
#define ep_world_distancejoint_count
// ep_world_distancejoint_count(world_id)
external_call(global.define_ep_world_distancejoint_count,argument0);
#define ep_world_first_railjoint
// ep_world_first_railjoint(world_id)
external_call(global.define_ep_world_first_railjoint,argument0);
#define ep_world_last_railjoint
// ep_world_last_railjoint(world_id)
external_call(global.define_ep_world_last_railjoint,argument0);
#define ep_world_railjoint_count
// ep_world_railjoint_count(world_id)
external_call(global.define_ep_world_railjoint_count,argument0);
#define ep_world_first_sliderjoint
// ep_world_first_sliderjoint(world_id)
external_call(global.define_ep_world_first_sliderjoint,argument0);
#define ep_world_last_sliderjoint
// ep_world_last_sliderjoint(world_id)
external_call(global.define_ep_world_last_sliderjoint,argument0);
#define ep_world_sliderjoint_count
// ep_world_sliderjoint_count(world_id)
external_call(global.define_ep_world_sliderjoint_count,argument0);
#define ep_world_first_view
// ep_world_first_view(world_id)
external_call(global.define_ep_world_first_view,argument0);
#define ep_world_last_view
// ep_world_last_view(world_id)
external_call(global.define_ep_world_last_view,argument0);
#define ep_world_view_count
// ep_world_view_count(world_id)
external_call(global.define_ep_world_view_count,argument0);
#define ep_world_first_water
// ep_world_first_water(world_id)
external_call(global.define_ep_world_first_water,argument0);
#define ep_world_last_water
// ep_world_last_water(world_id)
external_call(global.define_ep_world_last_water,argument0);
#define ep_world_water_count
// ep_world_water_count(world_id)
external_call(global.define_ep_world_water_count,argument0);
#define ep_world_shape_count
// ep_world_shape_count(world_id)
external_call(global.define_ep_world_shape_count,argument0);
#define ep_world_force_count
// ep_world_force_count(world_id)
external_call(global.define_ep_world_force_count,argument0);
#define ep_polygon_create
// ep_polygon_create(world_id,vertex_count)
external_call(global.define_ep_polygon_create,argument0,argument1);
#define ep_polygon_destroy
// ep_polygon_destroy(world_id,polygon_id)
external_call(global.define_ep_polygon_destroy,argument0,argument1);
#define ep_polygon_exists
// ep_polygon_exists(world_id,polygon_id)
external_call(global.define_ep_polygon_exists,argument0,argument1);
#define ep_polygon_set_vertex
// ep_polygon_set_vertex(world_id,polygon_id,index,x,y)
external_call(global.define_ep_polygon_set_vertex,argument0,argument1,argument2,argument3,argument4);
#define ep_polygon_initialize
// ep_polygon_initialize(world_id,polygon_id)
external_call(global.define_ep_polygon_initialize,argument0,argument1);
#define ep_polygon_get_vertex_count
// ep_polygon_get_vertex_count(world_id,polygon_id)
external_call(global.define_ep_polygon_get_vertex_count,argument0,argument1);
#define ep_polygon_get_vertex_x
// ep_polygon_get_vertex_x(world_id,polygon_id,index)
external_call(global.define_ep_polygon_get_vertex_x,argument0,argument1,argument2);
#define ep_polygon_get_vertex_y
// ep_polygon_get_vertex_y(world_id,polygon_id,index)
external_call(global.define_ep_polygon_get_vertex_y,argument0,argument1,argument2);
#define ep_polygon_get_vertex_normal_x
// ep_polygon_get_vertex_normal_x(world_id,polygon_id,index)
external_call(global.define_ep_polygon_get_vertex_normal_x,argument0,argument1,argument2);
#define ep_polygon_get_vertex_normal_y
// ep_polygon_get_vertex_normal_y(world_id,polygon_id,index)
external_call(global.define_ep_polygon_get_vertex_normal_y,argument0,argument1,argument2);
#define ep_polygon_get_edge_length
// ep_polygon_get_edge_length(world_id,polygon_id,index)
external_call(global.define_ep_polygon_get_edge_length,argument0,argument1,argument2);
#define ep_polygon_previous
// ep_polygon_previous(world_id,polygon_id)
external_call(global.define_ep_polygon_previous,argument0,argument1);
#define ep_polygon_next
// ep_polygon_next(world_id,polygon_id)
external_call(global.define_ep_polygon_next,argument0,argument1);
#define ep_polygon_set_uservar
// ep_polygon_set_uservar(world_id,polygon_id,index,value)
external_call(global.define_ep_polygon_set_uservar,argument0,argument1,argument2,argument3);
#define ep_polygon_get_uservar
// ep_polygon_get_uservar(world_id,polygon_id,index)
external_call(global.define_ep_polygon_get_uservar,argument0,argument1,argument2);
#define ep_body_create_static
// ep_body_create_static(world_id)
external_call(global.define_ep_body_create_static,argument0);
#define ep_body_create_dynamic
// ep_body_create_dynamic(world_id,norotation)
external_call(global.define_ep_body_create_dynamic,argument0,argument1);
#define ep_body_destroy
// ep_body_destroy(world_id,body_id)
external_call(global.define_ep_body_destroy,argument0,argument1);
#define ep_body_exists
// ep_body_exists(world_id,body_id)
external_call(global.define_ep_body_exists,argument0,argument1);
#define ep_body_get_first_hingejoint
// ep_body_get_first_hingejoint(world_id,body_id)
external_call(global.define_ep_body_get_first_hingejoint,argument0,argument1);
#define ep_body_get_last_hingejoint
// ep_body_get_last_hingejoint(world_id,body_id)
external_call(global.define_ep_body_get_last_hingejoint,argument0,argument1);
#define ep_body_get_previous_hingejoint
// ep_body_get_previous_hingejoint(world_id,body_id,hingejoint_id)
external_call(global.define_ep_body_get_previous_hingejoint,argument0,argument1,argument2);
#define ep_body_get_next_hingejoint
// ep_body_get_next_hingejoint(world_id,body_id,hingejoint_id)
external_call(global.define_ep_body_get_next_hingejoint,argument0,argument1,argument2);
#define ep_body_get_first_distancejoint
// ep_body_get_first_distancejoint(world_id,body_id)
external_call(global.define_ep_body_get_first_distancejoint,argument0,argument1);
#define ep_body_get_last_distancejoint
// ep_body_get_last_distancejoint(world_id,body_id)
external_call(global.define_ep_body_get_last_distancejoint,argument0,argument1);
#define ep_body_get_previous_distancejoint
// ep_body_get_previous_distancejoint(world_id,body_id,distancejoint_id)
external_call(global.define_ep_body_get_previous_distancejoint,argument0,argument1,argument2);
#define ep_body_get_next_distancejoint
// ep_body_get_next_distancejoint(world_id,body_id,distancejoint_id)
external_call(global.define_ep_body_get_next_distancejoint,argument0,argument1,argument2);
#define ep_body_get_first_railjoint
// ep_body_get_first_railjoint(world_id,body_id)
external_call(global.define_ep_body_get_first_railjoint,argument0,argument1);
#define ep_body_get_last_railjoint
// ep_body_get_last_railjoint(world_id,body_id)
external_call(global.define_ep_body_get_last_railjoint,argument0,argument1);
#define ep_body_get_previous_railjoint
// ep_body_get_previous_railjoint(world_id,body_id,railjoint_id)
external_call(global.define_ep_body_get_previous_railjoint,argument0,argument1,argument2);
#define ep_body_get_next_railjoint
// ep_body_get_next_railjoint(world_id,body_id,railjoint_id)
external_call(global.define_ep_body_get_next_railjoint,argument0,argument1,argument2);
#define ep_body_get_first_sliderjoint
// ep_body_get_first_sliderjoint(world_id,body_id)
external_call(global.define_ep_body_get_first_sliderjoint,argument0,argument1);
#define ep_body_get_last_sliderjoint
// ep_body_get_last_sliderjoint(world_id,body_id)
external_call(global.define_ep_body_get_last_sliderjoint,argument0,argument1);
#define ep_body_get_previous_sliderjoint
// ep_body_get_previous_sliderjoint(world_id,body_id,sliderjoint_id)
external_call(global.define_ep_body_get_previous_sliderjoint,argument0,argument1,argument2);
#define ep_body_get_next_sliderjoint
// ep_body_get_next_sliderjoint(world_id,body_id,sliderjoint_id)
external_call(global.define_ep_body_get_next_sliderjoint,argument0,argument1,argument2);
#define ep_body_calculate_mass
// ep_body_calculate_mass(world_id,body_id)
external_call(global.define_ep_body_calculate_mass,argument0,argument1);
#define ep_body_set_mass
// ep_body_set_mass(world_id,body_id,mass)
external_call(global.define_ep_body_set_mass,argument0,argument1,argument2);
#define ep_body_set_inertia
// ep_body_set_inertia(world_id,body_id,inertia)
external_call(global.define_ep_body_set_inertia,argument0,argument1,argument2);
#define ep_body_set_center
// ep_body_set_center(world_id,body_id,localx,localy,updateinertia)
external_call(global.define_ep_body_set_center,argument0,argument1,argument2,argument3,argument4);
#define ep_body_set_position
// ep_body_set_position(world_id,body_id,x,y,rot)
external_call(global.define_ep_body_set_position,argument0,argument1,argument2,argument3,argument4);
#define ep_body_set_position_center
// ep_body_set_position_center(world_id,body_id,x,y,rot)
external_call(global.define_ep_body_set_position_center,argument0,argument1,argument2,argument3,argument4);
#define ep_body_set_position_local_point
// ep_body_set_position_local_point(world_id,body_id,x,y,rot,localx,localy)
external_call(global.define_ep_body_set_position_local_point,argument0,argument1,argument2,argument3,argument4,argument5,argument6);
#define ep_body_set_velocity_center
// ep_body_set_velocity_center(world_id,body_id,xvel,yvel,rotvel)
external_call(global.define_ep_body_set_velocity_center,argument0,argument1,argument2,argument3,argument4);
#define ep_body_set_velocity_local_point
// ep_body_set_velocity_local_point(world_id,body_id,xvel,yvel,rotvel,localx,localy)
external_call(global.define_ep_body_set_velocity_local_point,argument0,argument1,argument2,argument3,argument4,argument5,argument6);
#define ep_body_set_max_velocity
// ep_body_set_max_velocity(world_id,body_id,maxvel,maxrotvel)
external_call(global.define_ep_body_set_max_velocity,argument0,argument1,argument2,argument3);
#define ep_body_set_gravity
// ep_body_set_gravity(world_id,body_id,gravity_x,gravity_y)
external_call(global.define_ep_body_set_gravity,argument0,argument1,argument2,argument3);
#define ep_body_set_damping
// ep_body_set_damping(world_id,body_id,damping,rotdamping)
external_call(global.define_ep_body_set_damping,argument0,argument1,argument2,argument3);
#define ep_body_store_impulses
// ep_body_store_impulses(world_id,body_id,storecontactimpulses,storejointimpulses)
external_call(global.define_ep_body_store_impulses,argument0,argument1,argument2,argument3);
#define ep_body_set_sleeping
// ep_body_set_sleeping(world_id,body_id,sleepstable,sleepoutofview)
external_call(global.define_ep_body_set_sleeping,argument0,argument1,argument2,argument3);
#define ep_body_collision_test_box
// ep_body_collision_test_box(world_id,body_id,w,h,x,y,rot,contact_threshold,collidemask1,collidemask2,group)
external_call(global.define_ep_body_collision_test_box,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8,argument9,argument10);
#define ep_body_collision_test_line
// ep_body_collision_test_line(world_id,body_id,x1,y1,x2,y2,contact_threshold,collidemask1,collidemask2,group)
external_call(global.define_ep_body_collision_test_line,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8,argument9);
#define ep_body_collision_test_circle
// ep_body_collision_test_circle(world_id,body_id,r,x,y,contact_threshold,collidemask1,collidemask2,group)
external_call(global.define_ep_body_collision_test_circle,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_body_collision_test_polygon
// ep_body_collision_test_polygon(world_id,body_id,polygon_id,x,y,rot,contact_threshold,collidemask1,collidemask2,group)
external_call(global.define_ep_body_collision_test_polygon,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8,argument9);
#define ep_body_ray_cast
// ep_body_ray_cast(world_id,body_id,x,y,vx,vy,collidemask1,collidemask2,group)
external_call(global.define_ep_body_ray_cast,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_body_apply_impulse
// ep_body_apply_impulse(world_id,body_id,localx,localy,xforce,yforce,torque,local,ignoremass,awake)
external_call(global.define_ep_body_apply_impulse,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8,argument9);
#define ep_body_apply_impulse_relative
// ep_body_apply_impulse_relative(world_id,body_id,relativex,relativey,xforce,yforce,torque,ignoremass,awake)
external_call(global.define_ep_body_apply_impulse_relative,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_body_get_mass
// ep_body_get_mass(world_id,body_id)
external_call(global.define_ep_body_get_mass,argument0,argument1);
#define ep_body_get_inertia
// ep_body_get_inertia(world_id,body_id)
external_call(global.define_ep_body_get_inertia,argument0,argument1);
#define ep_body_get_center_of_mass_x
// ep_body_get_center_of_mass_x(world_id,body_id)
external_call(global.define_ep_body_get_center_of_mass_x,argument0,argument1);
#define ep_body_get_center_of_mass_y
// ep_body_get_center_of_mass_y(world_id,body_id)
external_call(global.define_ep_body_get_center_of_mass_y,argument0,argument1);
#define ep_body_get_x
// ep_body_get_x(world_id,body_id)
external_call(global.define_ep_body_get_x,argument0,argument1);
#define ep_body_get_y
// ep_body_get_y(world_id,body_id)
external_call(global.define_ep_body_get_y,argument0,argument1);
#define ep_body_get_x_center
// ep_body_get_x_center(world_id,body_id)
external_call(global.define_ep_body_get_x_center,argument0,argument1);
#define ep_body_get_y_center
// ep_body_get_y_center(world_id,body_id)
external_call(global.define_ep_body_get_y_center,argument0,argument1);
#define ep_body_get_rot
// ep_body_get_rot(world_id,body_id)
external_call(global.define_ep_body_get_rot,argument0,argument1);
#define ep_body_get_xvel_center
// ep_body_get_xvel_center(world_id,body_id)
external_call(global.define_ep_body_get_xvel_center,argument0,argument1);
#define ep_body_get_yvel_center
// ep_body_get_yvel_center(world_id,body_id)
external_call(global.define_ep_body_get_yvel_center,argument0,argument1);
#define ep_body_get_xvel_local_point
// ep_body_get_xvel_local_point(world_id,body_id,localx,localy)
external_call(global.define_ep_body_get_xvel_local_point,argument0,argument1,argument2,argument3);
#define ep_body_get_yvel_local_point
// ep_body_get_yvel_local_point(world_id,body_id,localx,localy)
external_call(global.define_ep_body_get_yvel_local_point,argument0,argument1,argument2,argument3);
#define ep_body_get_rotvel
// ep_body_get_rotvel(world_id,body_id)
external_call(global.define_ep_body_get_rotvel,argument0,argument1);
#define ep_body_is_static
// ep_body_is_static(world_id,body_id)
external_call(global.define_ep_body_is_static,argument0,argument1);
#define ep_body_is_norotation
// ep_body_is_norotation(world_id,body_id)
external_call(global.define_ep_body_is_norotation,argument0,argument1);
#define ep_body_is_sleeping
// ep_body_is_sleeping(world_id,body_id)
external_call(global.define_ep_body_is_sleeping,argument0,argument1);
#define ep_body_stable_timer
// ep_body_stable_timer(world_id,body_id)
external_call(global.define_ep_body_stable_timer,argument0,argument1);
#define ep_body_out_of_view_timer
// ep_body_out_of_view_timer(world_id,body_id)
external_call(global.define_ep_body_out_of_view_timer,argument0,argument1);
#define ep_body_coord_local_to_world_x
// ep_body_coord_local_to_world_x(world_id,body_id,localx,localy)
external_call(global.define_ep_body_coord_local_to_world_x,argument0,argument1,argument2,argument3);
#define ep_body_coord_local_to_world_y
// ep_body_coord_local_to_world_y(world_id,body_id,localx,localy)
external_call(global.define_ep_body_coord_local_to_world_y,argument0,argument1,argument2,argument3);
#define ep_body_coord_world_to_local_x
// ep_body_coord_world_to_local_x(world_id,body_id,worldx,worldy)
external_call(global.define_ep_body_coord_world_to_local_x,argument0,argument1,argument2,argument3);
#define ep_body_coord_world_to_local_y
// ep_body_coord_world_to_local_y(world_id,body_id,worldx,worldy)
external_call(global.define_ep_body_coord_world_to_local_y,argument0,argument1,argument2,argument3);
#define ep_body_vect_local_to_world_x
// ep_body_vect_local_to_world_x(world_id,body_id,vx,vy)
external_call(global.define_ep_body_vect_local_to_world_x,argument0,argument1,argument2,argument3);
#define ep_body_vect_local_to_world_y
// ep_body_vect_local_to_world_y(world_id,body_id,vx,vy)
external_call(global.define_ep_body_vect_local_to_world_y,argument0,argument1,argument2,argument3);
#define ep_body_vect_world_to_local_x
// ep_body_vect_world_to_local_x(world_id,body_id,vx,vy)
external_call(global.define_ep_body_vect_world_to_local_x,argument0,argument1,argument2,argument3);
#define ep_body_vect_world_to_local_y
// ep_body_vect_world_to_local_y(world_id,body_id,vx,vy)
external_call(global.define_ep_body_vect_world_to_local_y,argument0,argument1,argument2,argument3);
#define ep_body_boxchain_begin
// ep_body_boxchain_begin(world_id,body_id,vertexcount)
external_call(global.define_ep_body_boxchain_begin,argument0,argument1,argument2);
#define ep_body_boxchain_end
// ep_body_boxchain_end(world_id,body_id,circular,ignorefirstlast,width_top,width_bottom,density)
external_call(global.define_ep_body_boxchain_end,argument0,argument1,argument2,argument3,argument4,argument5,argument6);
#define ep_body_boxchain_set_vertex
// ep_body_boxchain_set_vertex(world_id,body_id,index,x,y)
external_call(global.define_ep_body_boxchain_set_vertex,argument0,argument1,argument2,argument3,argument4);
#define ep_body_boxchain_get_first
// ep_body_boxchain_get_first(world_id,body_id)
external_call(global.define_ep_body_boxchain_get_first,argument0,argument1);
#define ep_body_boxchain_get_last
// ep_body_boxchain_get_last(world_id,body_id)
external_call(global.define_ep_body_boxchain_get_last,argument0,argument1);
#define ep_body_previous
// ep_body_previous(world_id,body_id)
external_call(global.define_ep_body_previous,argument0,argument1);
#define ep_body_next
// ep_body_next(world_id,body_id)
external_call(global.define_ep_body_next,argument0,argument1);
#define ep_body_set_uservar
// ep_body_set_uservar(world_id,body_id,index,value)
external_call(global.define_ep_body_set_uservar,argument0,argument1,argument2,argument3);
#define ep_body_get_uservar
// ep_body_get_uservar(world_id,body_id,index)
external_call(global.define_ep_body_get_uservar,argument0,argument1,argument2);
#define ep_body_first_shape
// ep_body_first_shape(world_id,body_id)
external_call(global.define_ep_body_first_shape,argument0,argument1);
#define ep_body_last_shape
// ep_body_last_shape(world_id,body_id)
external_call(global.define_ep_body_last_shape,argument0,argument1);
#define ep_body_shape_count
// ep_body_shape_count(world_id,body_id)
external_call(global.define_ep_body_shape_count,argument0,argument1);
#define ep_body_first_force
// ep_body_first_force(world_id,body_id)
external_call(global.define_ep_body_first_force,argument0,argument1);
#define ep_body_last_force
// ep_body_last_force(world_id,body_id)
external_call(global.define_ep_body_last_force,argument0,argument1);
#define ep_body_force_count
// ep_body_force_count(world_id,body_id)
external_call(global.define_ep_body_force_count,argument0,argument1);
#define ep_contact_destroy
// ep_contact_destroy(world_id,contact_id)
external_call(global.define_ep_contact_destroy,argument0,argument1);
#define ep_contact_exists
// ep_contact_exists(world_id,contact_id)
external_call(global.define_ep_contact_exists,argument0,argument1);
#define ep_contact_get_body1
// ep_contact_get_body1(world_id,contact_id)
external_call(global.define_ep_contact_get_body1,argument0,argument1);
#define ep_contact_get_body2
// ep_contact_get_body2(world_id,contact_id)
external_call(global.define_ep_contact_get_body2,argument0,argument1);
#define ep_contact_get_shape1
// ep_contact_get_shape1(world_id,contact_id)
external_call(global.define_ep_contact_get_shape1,argument0,argument1);
#define ep_contact_get_shape2
// ep_contact_get_shape2(world_id,contact_id)
external_call(global.define_ep_contact_get_shape2,argument0,argument1);
#define ep_contact_get_normal_x
// ep_contact_get_normal_x(world_id,contact_id)
external_call(global.define_ep_contact_get_normal_x,argument0,argument1);
#define ep_contact_get_normal_y
// ep_contact_get_normal_y(world_id,contact_id)
external_call(global.define_ep_contact_get_normal_y,argument0,argument1);
#define ep_contact_get_point1_active
// ep_contact_get_point1_active(world_id,contact_id)
external_call(global.define_ep_contact_get_point1_active,argument0,argument1);
#define ep_contact_get_point2_active
// ep_contact_get_point2_active(world_id,contact_id)
external_call(global.define_ep_contact_get_point2_active,argument0,argument1);
#define ep_contact_get_point1_x
// ep_contact_get_point1_x(world_id,contact_id)
external_call(global.define_ep_contact_get_point1_x,argument0,argument1);
#define ep_contact_get_point1_y
// ep_contact_get_point1_y(world_id,contact_id)
external_call(global.define_ep_contact_get_point1_y,argument0,argument1);
#define ep_contact_get_point2_x
// ep_contact_get_point2_x(world_id,contact_id)
external_call(global.define_ep_contact_get_point2_x,argument0,argument1);
#define ep_contact_get_point2_y
// ep_contact_get_point2_y(world_id,contact_id)
external_call(global.define_ep_contact_get_point2_y,argument0,argument1);
#define ep_contact_get_point1_separation
// ep_contact_get_point1_separation(world_id,contact_id)
external_call(global.define_ep_contact_get_point1_separation,argument0,argument1);
#define ep_contact_get_point2_separation
// ep_contact_get_point2_separation(world_id,contact_id)
external_call(global.define_ep_contact_get_point2_separation,argument0,argument1);
#define ep_contact_get_point1_normalforce
// ep_contact_get_point1_normalforce(world_id,contact_id)
external_call(global.define_ep_contact_get_point1_normalforce,argument0,argument1);
#define ep_contact_get_point1_tangentforce
// ep_contact_get_point1_tangentforce(world_id,contact_id)
external_call(global.define_ep_contact_get_point1_tangentforce,argument0,argument1);
#define ep_contact_get_point2_normalforce
// ep_contact_get_point2_normalforce(world_id,contact_id)
external_call(global.define_ep_contact_get_point2_normalforce,argument0,argument1);
#define ep_contact_get_point2_tangentforce
// ep_contact_get_point2_tangentforce(world_id,contact_id)
external_call(global.define_ep_contact_get_point2_tangentforce,argument0,argument1);
#define ep_contact_get_point1_normalveldelta
// ep_contact_get_point1_normalveldelta(world_id,contact_id)
external_call(global.define_ep_contact_get_point1_normalveldelta,argument0,argument1);
#define ep_contact_get_point1_tangentveldelta
// ep_contact_get_point1_tangentveldelta(world_id,contact_id)
external_call(global.define_ep_contact_get_point1_tangentveldelta,argument0,argument1);
#define ep_contact_get_point2_normalveldelta
// ep_contact_get_point2_normalveldelta(world_id,contact_id)
external_call(global.define_ep_contact_get_point2_normalveldelta,argument0,argument1);
#define ep_contact_get_point2_tangentveldelta
// ep_contact_get_point2_tangentveldelta(world_id,contact_id)
external_call(global.define_ep_contact_get_point2_tangentveldelta,argument0,argument1);
#define ep_contact_previous
// ep_contact_previous(world_id,contact_id)
external_call(global.define_ep_contact_previous,argument0,argument1);
#define ep_contact_next
// ep_contact_next(world_id,contact_id)
external_call(global.define_ep_contact_next,argument0,argument1);
#define ep_hingejoint_create
// ep_hingejoint_create(world_id,body1_id,body2_id,x1,y1,x2,y2,referencerotation)
external_call(global.define_ep_hingejoint_create,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7);
#define ep_hingejoint_destroy
// ep_hingejoint_destroy(world_id,hingejoint_id)
external_call(global.define_ep_hingejoint_destroy,argument0,argument1);
#define ep_hingejoint_exists
// ep_hingejoint_exists(world_id,hingejoint_id)
external_call(global.define_ep_hingejoint_exists,argument0,argument1);
#define ep_hingejoint_set_max_force
// ep_hingejoint_set_max_force(world_id,hingejoint_id,maxforce)
external_call(global.define_ep_hingejoint_set_max_force,argument0,argument1,argument2);
#define ep_hingejoint_set_motor
// ep_hingejoint_set_motor(world_id,hingejoint_id,maxmotortorque,motorvel)
external_call(global.define_ep_hingejoint_set_motor,argument0,argument1,argument2,argument3);
#define ep_hingejoint_set_limit_settings
// ep_hingejoint_set_limit_settings(world_id,hingejoint_id,contact_threshold,velocity_threshold)
external_call(global.define_ep_hingejoint_set_limit_settings,argument0,argument1,argument2,argument3);
#define ep_hingejoint_set_lower_limit
// ep_hingejoint_set_lower_limit(world_id,hingejoint_id,maxlimittorque,rotation,restitution,velocity)
external_call(global.define_ep_hingejoint_set_lower_limit,argument0,argument1,argument2,argument3,argument4,argument5);
#define ep_hingejoint_set_upper_limit
// ep_hingejoint_set_upper_limit(world_id,hingejoint_id,maxlimittorque,rotation,restitution,velocity)
external_call(global.define_ep_hingejoint_set_upper_limit,argument0,argument1,argument2,argument3,argument4,argument5);
#define ep_hingejoint_set_lower_spring
// ep_hingejoint_set_lower_spring(world_id,hingejoint_id,k,rotation,damping)
external_call(global.define_ep_hingejoint_set_lower_spring,argument0,argument1,argument2,argument3,argument4);
#define ep_hingejoint_set_upper_spring
// ep_hingejoint_set_upper_spring(world_id,hingejoint_id,k,rotation,damping)
external_call(global.define_ep_hingejoint_set_upper_spring,argument0,argument1,argument2,argument3,argument4);
#define ep_hingejoint_get_body1
// ep_hingejoint_get_body1(world_id,hingejoint_id)
external_call(global.define_ep_hingejoint_get_body1,argument0,argument1);
#define ep_hingejoint_get_body2
// ep_hingejoint_get_body2(world_id,hingejoint_id)
external_call(global.define_ep_hingejoint_get_body2,argument0,argument1);
#define ep_hingejoint_get_rotation
// ep_hingejoint_get_rotation(world_id,hingejoint_id)
external_call(global.define_ep_hingejoint_get_rotation,argument0,argument1);
#define ep_hingejoint_get_xforce
// ep_hingejoint_get_xforce(world_id,hingejoint_id)
external_call(global.define_ep_hingejoint_get_xforce,argument0,argument1);
#define ep_hingejoint_get_yforce
// ep_hingejoint_get_yforce(world_id,hingejoint_id)
external_call(global.define_ep_hingejoint_get_yforce,argument0,argument1);
#define ep_hingejoint_get_motor_torque
// ep_hingejoint_get_motor_torque(world_id,hingejoint_id)
external_call(global.define_ep_hingejoint_get_motor_torque,argument0,argument1);
#define ep_hingejoint_get_limit_torque
// ep_hingejoint_get_limit_torque(world_id,hingejoint_id)
external_call(global.define_ep_hingejoint_get_limit_torque,argument0,argument1);
#define ep_hingejoint_previous
// ep_hingejoint_previous(world_id,hingejoint_id)
external_call(global.define_ep_hingejoint_previous,argument0,argument1);
#define ep_hingejoint_next
// ep_hingejoint_next(world_id,hingejoint_id)
external_call(global.define_ep_hingejoint_next,argument0,argument1);
#define ep_hingejoint_set_uservar
// ep_hingejoint_set_uservar(world_id,hingejoint_id,index,value)
external_call(global.define_ep_hingejoint_set_uservar,argument0,argument1,argument2,argument3);
#define ep_hingejoint_get_uservar
// ep_hingejoint_get_uservar(world_id,hingejoint_id,index)
external_call(global.define_ep_hingejoint_get_uservar,argument0,argument1,argument2);
#define ep_distancejoint_create
// ep_distancejoint_create(world_id,body1_id,body2_id,x1,y1,x2,y2)
external_call(global.define_ep_distancejoint_create,argument0,argument1,argument2,argument3,argument4,argument5,argument6);
#define ep_distancejoint_destroy
// ep_distancejoint_destroy(world_id,distancejoint_id)
external_call(global.define_ep_distancejoint_destroy,argument0,argument1);
#define ep_distancejoint_exists
// ep_distancejoint_exists(world_id,distancejoint_id)
external_call(global.define_ep_distancejoint_exists,argument0,argument1);
#define ep_distancejoint_set_motor
// ep_distancejoint_set_motor(world_id,distancejoint_id,maxmotorforce,motorvel)
external_call(global.define_ep_distancejoint_set_motor,argument0,argument1,argument2,argument3);
#define ep_distancejoint_set_limit_settings
// ep_distancejoint_set_limit_settings(world_id,distancejoint_id,contact_threshold,velocity_threshold)
external_call(global.define_ep_distancejoint_set_limit_settings,argument0,argument1,argument2,argument3);
#define ep_distancejoint_set_lower_limit
// ep_distancejoint_set_lower_limit(world_id,distancejoint_id,maxlimitforce,distance,restitution,velocity)
external_call(global.define_ep_distancejoint_set_lower_limit,argument0,argument1,argument2,argument3,argument4,argument5);
#define ep_distancejoint_set_upper_limit
// ep_distancejoint_set_upper_limit(world_id,distancejoint_id,maxlimitforce,distance,restitution,velocity)
external_call(global.define_ep_distancejoint_set_upper_limit,argument0,argument1,argument2,argument3,argument4,argument5);
#define ep_distancejoint_set_lower_spring
// ep_distancejoint_set_lower_spring(world_id,distancejoint_id,k,distance,damping)
external_call(global.define_ep_distancejoint_set_lower_spring,argument0,argument1,argument2,argument3,argument4);
#define ep_distancejoint_set_upper_spring
// ep_distancejoint_set_upper_spring(world_id,distancejoint_id,k,distance,damping)
external_call(global.define_ep_distancejoint_set_upper_spring,argument0,argument1,argument2,argument3,argument4);
#define ep_distancejoint_get_body1
// ep_distancejoint_get_body1(world_id,distancejoint_id)
external_call(global.define_ep_distancejoint_get_body1,argument0,argument1);
#define ep_distancejoint_get_body2
// ep_distancejoint_get_body2(world_id,distancejoint_id)
external_call(global.define_ep_distancejoint_get_body2,argument0,argument1);
#define ep_distancejoint_get_distance
// ep_distancejoint_get_distance(world_id,distancejoint_id)
external_call(global.define_ep_distancejoint_get_distance,argument0,argument1);
#define ep_distancejoint_get_motor_force
// ep_distancejoint_get_motor_force(world_id,distancejoint_id)
external_call(global.define_ep_distancejoint_get_motor_force,argument0,argument1);
#define ep_distancejoint_get_limit_force
// ep_distancejoint_get_limit_force(world_id,distancejoint_id)
external_call(global.define_ep_distancejoint_get_limit_force,argument0,argument1);
#define ep_distancejoint_previous
// ep_distancejoint_previous(world_id,distancejoint_id)
external_call(global.define_ep_distancejoint_previous,argument0,argument1);
#define ep_distancejoint_next
// ep_distancejoint_next(world_id,distancejoint_id)
external_call(global.define_ep_distancejoint_next,argument0,argument1);
#define ep_distancejoint_set_uservar
// ep_distancejoint_set_uservar(world_id,distancejoint_id,index,value)
external_call(global.define_ep_distancejoint_set_uservar,argument0,argument1,argument2,argument3);
#define ep_distancejoint_get_uservar
// ep_distancejoint_get_uservar(world_id,distancejoint_id,index)
external_call(global.define_ep_distancejoint_get_uservar,argument0,argument1,argument2);
#define ep_railjoint_create
// ep_railjoint_create(world_id,body1_id,body2_id,x1,y1,x2a,y2a,x2b,y2b)
external_call(global.define_ep_railjoint_create,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_railjoint_destroy
// ep_railjoint_destroy(world_id,railjoint_id)
external_call(global.define_ep_railjoint_destroy,argument0,argument1);
#define ep_railjoint_exists
// ep_railjoint_exists(world_id,railjoint_id)
external_call(global.define_ep_railjoint_exists,argument0,argument1);
#define ep_railjoint_set_max_normal_force
// ep_railjoint_set_max_normal_force(world_id,railjoint_id,maxnormalforce)
external_call(global.define_ep_railjoint_set_max_normal_force,argument0,argument1,argument2);
#define ep_railjoint_set_motor
// ep_railjoint_set_motor(world_id,railjoint_id,maxmotorforce,motorvel)
external_call(global.define_ep_railjoint_set_motor,argument0,argument1,argument2,argument3);
#define ep_railjoint_set_limit_settings
// ep_railjoint_set_limit_settings(world_id,railjoint_id,contact_threshold,velocity_threshold)
external_call(global.define_ep_railjoint_set_limit_settings,argument0,argument1,argument2,argument3);
#define ep_railjoint_set_lower_limit
// ep_railjoint_set_lower_limit(world_id,railjoint_id,maxlimitforce,position,restitution,velocity)
external_call(global.define_ep_railjoint_set_lower_limit,argument0,argument1,argument2,argument3,argument4,argument5);
#define ep_railjoint_set_upper_limit
// ep_railjoint_set_upper_limit(world_id,railjoint_id,maxlimitforce,position,restitution,velocity)
external_call(global.define_ep_railjoint_set_upper_limit,argument0,argument1,argument2,argument3,argument4,argument5);
#define ep_railjoint_set_lower_spring
// ep_railjoint_set_lower_spring(world_id,railjoint_id,k,position,damping)
external_call(global.define_ep_railjoint_set_lower_spring,argument0,argument1,argument2,argument3,argument4);
#define ep_railjoint_set_upper_spring
// ep_railjoint_set_upper_spring(world_id,railjoint_id,k,position,damping)
external_call(global.define_ep_railjoint_set_upper_spring,argument0,argument1,argument2,argument3,argument4);
#define ep_railjoint_get_body1
// ep_railjoint_get_body1(world_id,railjoint_id)
external_call(global.define_ep_railjoint_get_body1,argument0,argument1);
#define ep_railjoint_get_body2
// ep_railjoint_get_body2(world_id,railjoint_id)
external_call(global.define_ep_railjoint_get_body2,argument0,argument1);
#define ep_railjoint_get_position
// ep_railjoint_get_position(world_id,railjoint_id)
external_call(global.define_ep_railjoint_get_position,argument0,argument1);
#define ep_railjoint_get_normal_force
// ep_railjoint_get_normal_force(world_id,railjoint_id)
external_call(global.define_ep_railjoint_get_normal_force,argument0,argument1);
#define ep_railjoint_get_motor_force
// ep_railjoint_get_motor_force(world_id,railjoint_id)
external_call(global.define_ep_railjoint_get_motor_force,argument0,argument1);
#define ep_railjoint_get_limit_force
// ep_railjoint_get_limit_force(world_id,railjoint_id)
external_call(global.define_ep_railjoint_get_limit_force,argument0,argument1);
#define ep_railjoint_previous
// ep_railjoint_previous(world_id,railjoint_id)
external_call(global.define_ep_railjoint_previous,argument0,argument1);
#define ep_railjoint_next
// ep_railjoint_next(world_id,railjoint_id)
external_call(global.define_ep_railjoint_next,argument0,argument1);
#define ep_railjoint_set_uservar
// ep_railjoint_set_uservar(world_id,railjoint_id,index,value)
external_call(global.define_ep_railjoint_set_uservar,argument0,argument1,argument2,argument3);
#define ep_railjoint_get_uservar
// ep_railjoint_get_uservar(world_id,railjoint_id,index)
external_call(global.define_ep_railjoint_get_uservar,argument0,argument1,argument2);
#define ep_sliderjoint_create
// ep_sliderjoint_create(world_id,body1_id,body2_id,x1,y1,x2a,y2a,x2b,y2b,rotation)
external_call(global.define_ep_sliderjoint_create,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8,argument9);
#define ep_sliderjoint_destroy
// ep_sliderjoint_destroy(world_id,sliderjoint_id)
external_call(global.define_ep_sliderjoint_destroy,argument0,argument1);
#define ep_sliderjoint_exists
// ep_sliderjoint_exists(world_id,sliderjoint_id)
external_call(global.define_ep_sliderjoint_exists,argument0,argument1);
#define ep_sliderjoint_set_max_combined_force
// ep_sliderjoint_set_max_combined_force(world_id,sliderjoint_id,maxnormalforce,torqueradius)
external_call(global.define_ep_sliderjoint_set_max_combined_force,argument0,argument1,argument2,argument3);
#define ep_sliderjoint_set_motor
// ep_sliderjoint_set_motor(world_id,sliderjoint_id,maxmotorforce,motorvel)
external_call(global.define_ep_sliderjoint_set_motor,argument0,argument1,argument2,argument3);
#define ep_sliderjoint_set_limit_settings
// ep_sliderjoint_set_limit_settings(world_id,sliderjoint_id,contact_threshold,velocity_threshold)
external_call(global.define_ep_sliderjoint_set_limit_settings,argument0,argument1,argument2,argument3);
#define ep_sliderjoint_set_lower_limit
// ep_sliderjoint_set_lower_limit(world_id,sliderjoint_id,maxlimitforce,position,restitution,velocity)
external_call(global.define_ep_sliderjoint_set_lower_limit,argument0,argument1,argument2,argument3,argument4,argument5);
#define ep_sliderjoint_set_upper_limit
// ep_sliderjoint_set_upper_limit(world_id,sliderjoint_id,maxlimitforce,position,restitution,velocity)
external_call(global.define_ep_sliderjoint_set_upper_limit,argument0,argument1,argument2,argument3,argument4,argument5);
#define ep_sliderjoint_set_lower_spring
// ep_sliderjoint_set_lower_spring(world_id,sliderjoint_id,k,position,damping)
external_call(global.define_ep_sliderjoint_set_lower_spring,argument0,argument1,argument2,argument3,argument4);
#define ep_sliderjoint_set_upper_spring
// ep_sliderjoint_set_upper_spring(world_id,sliderjoint_id,k,position,damping)
external_call(global.define_ep_sliderjoint_set_upper_spring,argument0,argument1,argument2,argument3,argument4);
#define ep_sliderjoint_get_body1
// ep_sliderjoint_get_body1(world_id,sliderjoint_id)
external_call(global.define_ep_sliderjoint_get_body1,argument0,argument1);
#define ep_sliderjoint_get_body2
// ep_sliderjoint_get_body2(world_id,sliderjoint_id)
external_call(global.define_ep_sliderjoint_get_body2,argument0,argument1);
#define ep_sliderjoint_get_position
// ep_sliderjoint_get_position(world_id,sliderjoint_id)
external_call(global.define_ep_sliderjoint_get_position,argument0,argument1);
#define ep_sliderjoint_get_normal_force
// ep_sliderjoint_get_normal_force(world_id,sliderjoint_id)
external_call(global.define_ep_sliderjoint_get_normal_force,argument0,argument1);
#define ep_sliderjoint_get_torque
// ep_sliderjoint_get_torque(world_id,sliderjoint_id)
external_call(global.define_ep_sliderjoint_get_torque,argument0,argument1);
#define ep_sliderjoint_get_combined_force
// ep_sliderjoint_get_combined_force(world_id,sliderjoint_id)
external_call(global.define_ep_sliderjoint_get_combined_force,argument0,argument1);
#define ep_sliderjoint_get_motor_force
// ep_sliderjoint_get_motor_force(world_id,sliderjoint_id)
external_call(global.define_ep_sliderjoint_get_motor_force,argument0,argument1);
#define ep_sliderjoint_get_limit_force
// ep_sliderjoint_get_limit_force(world_id,sliderjoint_id)
external_call(global.define_ep_sliderjoint_get_limit_force,argument0,argument1);
#define ep_sliderjoint_previous
// ep_sliderjoint_previous(world_id,sliderjoint_id)
external_call(global.define_ep_sliderjoint_previous,argument0,argument1);
#define ep_sliderjoint_next
// ep_sliderjoint_next(world_id,sliderjoint_id)
external_call(global.define_ep_sliderjoint_next,argument0,argument1);
#define ep_sliderjoint_set_uservar
// ep_sliderjoint_set_uservar(world_id,sliderjoint_id,index,value)
external_call(global.define_ep_sliderjoint_set_uservar,argument0,argument1,argument2,argument3);
#define ep_sliderjoint_get_uservar
// ep_sliderjoint_get_uservar(world_id,sliderjoint_id,index)
external_call(global.define_ep_sliderjoint_get_uservar,argument0,argument1,argument2);
#define ep_view_create
// ep_view_create(world_id)
external_call(global.define_ep_view_create,argument0);
#define ep_view_destroy
// ep_view_destroy(world_id,view_id)
external_call(global.define_ep_view_destroy,argument0,argument1);
#define ep_view_exists
// ep_view_exists(world_id,view_id)
external_call(global.define_ep_view_exists,argument0,argument1);
#define ep_view_set_rectangle
// ep_view_set_rectangle(world_id,view_id,x1,y1,x2,y2)
external_call(global.define_ep_view_set_rectangle,argument0,argument1,argument2,argument3,argument4,argument5);
#define ep_view_previous
// ep_view_previous(world_id,view_id)
external_call(global.define_ep_view_previous,argument0,argument1);
#define ep_view_next
// ep_view_next(world_id,view_id)
external_call(global.define_ep_view_next,argument0,argument1);
#define ep_view_set_uservar
// ep_view_set_uservar(world_id,view_id,index,value)
external_call(global.define_ep_view_set_uservar,argument0,argument1,argument2,argument3);
#define ep_view_get_uservar
// ep_view_get_uservar(world_id,view_id,index)
external_call(global.define_ep_view_get_uservar,argument0,argument1,argument2);
#define ep_water_create
// ep_water_create(world_id)
external_call(global.define_ep_water_create,argument0);
#define ep_water_destroy
// ep_water_destroy(world_id,water_id)
external_call(global.define_ep_water_destroy,argument0,argument1);
#define ep_water_exists
// ep_water_exists(world_id,water_id)
external_call(global.define_ep_water_exists,argument0,argument1);
#define ep_water_set_parameters
// ep_water_set_parameters(world_id,water_id,density,lineardrag,quadraticdrag,xvel,yvel,gravity_x,gravity_y)
external_call(global.define_ep_water_set_parameters,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_water_set_rectangle
// ep_water_set_rectangle(world_id,water_id,x1,y1,x2,y2)
external_call(global.define_ep_water_set_rectangle,argument0,argument1,argument2,argument3,argument4,argument5);
#define ep_water_previous
// ep_water_previous(world_id,water_id)
external_call(global.define_ep_water_previous,argument0,argument1);
#define ep_water_next
// ep_water_next(world_id,water_id)
external_call(global.define_ep_water_next,argument0,argument1);
#define ep_water_set_uservar
// ep_water_set_uservar(world_id,water_id,index,value)
external_call(global.define_ep_water_set_uservar,argument0,argument1,argument2,argument3);
#define ep_water_get_uservar
// ep_water_get_uservar(world_id,water_id,index)
external_call(global.define_ep_water_get_uservar,argument0,argument1,argument2);
#define ep_shape_create_box
// ep_shape_create_box(world_id,body_id,w,h,x,y,rot,density)
external_call(global.define_ep_shape_create_box,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7);
#define ep_shape_create_line
// ep_shape_create_line(world_id,body_id,x1,y1,x2,y2,density)
external_call(global.define_ep_shape_create_line,argument0,argument1,argument2,argument3,argument4,argument5,argument6);
#define ep_shape_create_circle
// ep_shape_create_circle(world_id,body_id,r,x,y,rot,density)
external_call(global.define_ep_shape_create_circle,argument0,argument1,argument2,argument3,argument4,argument5,argument6);
#define ep_shape_create_polygon
// ep_shape_create_polygon(world_id,body_id,polygon_id,x,y,rot,density)
external_call(global.define_ep_shape_create_polygon,argument0,argument1,argument2,argument3,argument4,argument5,argument6);
#define ep_shape_destroy
// ep_shape_destroy(world_id,body_id,shape_id)
external_call(global.define_ep_shape_destroy,argument0,argument1,argument2);
#define ep_shape_exists
// ep_shape_exists(world_id,body_id,shape_id)
external_call(global.define_ep_shape_exists,argument0,argument1,argument2);
#define ep_shape_get_first_contact
// ep_shape_get_first_contact(world_id,body_id,shape_id)
external_call(global.define_ep_shape_get_first_contact,argument0,argument1,argument2);
#define ep_shape_get_last_contact
// ep_shape_get_last_contact(world_id,body_id,shape_id)
external_call(global.define_ep_shape_get_last_contact,argument0,argument1,argument2);
#define ep_shape_get_previous_contact
// ep_shape_get_previous_contact(world_id,body_id,shape_id,contact_id)
external_call(global.define_ep_shape_get_previous_contact,argument0,argument1,argument2,argument3);
#define ep_shape_get_next_contact
// ep_shape_get_next_contact(world_id,body_id,shape_id,contact_id)
external_call(global.define_ep_shape_get_next_contact,argument0,argument1,argument2,argument3);
#define ep_shape_set_material
// ep_shape_set_material(world_id,body_id,shape_id,restitution,friction,normalvelocity,tangentvelocity)
external_call(global.define_ep_shape_set_material,argument0,argument1,argument2,argument3,argument4,argument5,argument6);
#define ep_shape_set_collision
// ep_shape_set_collision(world_id,body_id,shape_id,collidemask1,collidemask2,group)
external_call(global.define_ep_shape_set_collision,argument0,argument1,argument2,argument3,argument4,argument5);
#define ep_shape_collision_test_box
// ep_shape_collision_test_box(world_id,body_id,shape_id,w,h,x,y,rot,contact_threshold)
external_call(global.define_ep_shape_collision_test_box,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7,argument8);
#define ep_shape_collision_test_line
// ep_shape_collision_test_line(world_id,body_id,shape_id,x1,y1,x2,y2,contact_threshold)
external_call(global.define_ep_shape_collision_test_line,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7);
#define ep_shape_collision_test_circle
// ep_shape_collision_test_circle(world_id,body_id,shape_id,r,x,y,contact_threshold)
external_call(global.define_ep_shape_collision_test_circle,argument0,argument1,argument2,argument3,argument4,argument5,argument6);
#define ep_shape_collision_test_polygon
// ep_shape_collision_test_polygon(world_id,body_id,shape_id,polygon_id,x,y,rot,contact_threshold)
external_call(global.define_ep_shape_collision_test_polygon,argument0,argument1,argument2,argument3,argument4,argument5,argument6,argument7);
#define ep_shape_ray_cast
// ep_shape_ray_cast(world_id,body_id,shape_id,x,y,vx,vy)
external_call(global.define_ep_shape_ray_cast,argument0,argument1,argument2,argument3,argument4,argument5,argument6);
#define ep_shape_previous
// ep_shape_previous(world_id,body_id,shape_id)
external_call(global.define_ep_shape_previous,argument0,argument1,argument2);
#define ep_shape_next
// ep_shape_next(world_id,body_id,shape_id)
external_call(global.define_ep_shape_next,argument0,argument1,argument2);
#define ep_shape_set_uservar
// ep_shape_set_uservar(world_id,body_id,shape_id,index,value)
external_call(global.define_ep_shape_set_uservar,argument0,argument1,argument2,argument3,argument4);
#define ep_shape_get_uservar
// ep_shape_get_uservar(world_id,body_id,shape_id,index)
external_call(global.define_ep_shape_get_uservar,argument0,argument1,argument2,argument3);
#define ep_force_create
// ep_force_create(world_id,body_id,x,y,local,ignoremass)
external_call(global.define_ep_force_create,argument0,argument1,argument2,argument3,argument4,argument5);
#define ep_force_destroy
// ep_force_destroy(world_id,body_id,force_id)
external_call(global.define_ep_force_destroy,argument0,argument1,argument2);
#define ep_force_exists
// ep_force_exists(world_id,body_id,force_id)
external_call(global.define_ep_force_exists,argument0,argument1,argument2);
#define ep_force_set_force
// ep_force_set_force(world_id,body_id,force_id,xforce,yforce,torque,awake)
external_call(global.define_ep_force_set_force,argument0,argument1,argument2,argument3,argument4,argument5,argument6);
#define ep_force_previous
// ep_force_previous(world_id,body_id,force_id)
external_call(global.define_ep_force_previous,argument0,argument1,argument2);
#define ep_force_next
// ep_force_next(world_id,body_id,force_id)
external_call(global.define_ep_force_next,argument0,argument1,argument2);
#define ep_force_set_uservar
// ep_force_set_uservar(world_id,body_id,force_id,index,value)
external_call(global.define_ep_force_set_uservar,argument0,argument1,argument2,argument3,argument4);
#define ep_force_get_uservar
// ep_force_get_uservar(world_id,body_id,force_id,index)
external_call(global.define_ep_force_get_uservar,argument0,argument1,argument2,argument3);
