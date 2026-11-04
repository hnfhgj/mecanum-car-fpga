set root [file normalize [file join [file dirname [info script]] ..]]
cd $root
open_project [file join $root free_navigation.gprj] -pn GW5AT-LV60PG484AC1/I0 -device_version B
set_option -top_module free_navigation_top
set_option -output_base_name free_navigation
set_option -use_sspi_as_gpio 1
set_option -place_option 2
set_option -route_option 2
set_option -timing_driven 1
set_option -user_code 000097F4
run all
set build_name free_navigation
source [file join $root pillar_nav timing_check.tcl]
exit
