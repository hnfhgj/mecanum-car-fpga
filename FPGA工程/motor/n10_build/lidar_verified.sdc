# Combined timing constraints. Gowin activates only one SDC per project.
create_clock -name sys_clk -period 20.000 [get_ports {clk}]
# Conservative evaluation envelope: PCLK <=50MHz, source data launched
# on falling edge (4740=20). 0..5ns arrival budget is a board-validation
# assumption, NOT a measured sensor/PCB timing guarantee.
create_clock -name camera_pclk -period 20.000 [get_ports {camera_pclk}]
set_input_delay -clock camera_pclk -clock_fall -max 5.000 [get_ports {camera_data[*] camera_href camera_vsync}]
set_input_delay -clock camera_pclk -clock_fall -min 0.000 [get_ports {camera_data[*] camera_href camera_vsync}]
# Only cut asynchronous arrivals at the FIRST synchronizer stage.
# Do not cut synchronizer stage0->stage1 or all clock-domain paths.
set_false_path -to [get_pins {u_camera/capture/enable_p_0_s0/D}]
set_false_path -to [get_pins {u_camera/capture/ack_p_0_s0/D}]
set_false_path -to [get_pins {u_camera/capture/req_c_0_s0/D}]
# Bundled ready_bank remains stable until the ACK. Bound routing delay
# to one system cycle; it is deliberately NOT made a false path.
set_max_delay -from [get_regs {u_camera/capture/ready_bank_s0}] -to [get_regs {u_camera/capture/front_bank_s0}] 20.000
# The system-clock-qualified camera_ready is asynchronous to sensor PCLK.
# These are ONLY the async CLEAR inputs of the existing two-flop reset
# synchronizer (async assert, two-PCLK-edge synchronous deassert).
# Keep sync_0/Q -> sync_1/D and sync_1/Q -> all downstream CLEAR paths timed.
set_false_path -from [get_regs {g_clock.u_clock/lock_release/sync_1_s0}] -to [get_pins {u_camera/capture/preset/sync_0_s0/CLEAR u_camera/capture/preset/sync_1_s0/CLEAR}]

