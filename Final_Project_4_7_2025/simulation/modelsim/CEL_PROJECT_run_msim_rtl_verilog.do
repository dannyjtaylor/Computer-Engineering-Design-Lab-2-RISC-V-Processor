transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -vlog01compat -work work +incdir+C:/Users/Domenic/Desktop/Quartus_Stuff/CodeGenerated/DE10_LITE/CEL_PROJECT/hdl {C:/Users/Domenic/Desktop/Quartus_Stuff/CodeGenerated/DE10_LITE/CEL_PROJECT/hdl/vga_controller.v}
vlog -vlog01compat -work work +incdir+C:/Users/Domenic/Desktop/Quartus_Stuff/CodeGenerated/DE10_LITE/CEL_PROJECT {C:/Users/Domenic/Desktop/Quartus_Stuff/CodeGenerated/DE10_LITE/CEL_PROJECT/CEL_PROJECT.v}
vlog -sv -work work +incdir+C:/Users/Domenic/Desktop/Quartus_Stuff/CodeGenerated/DE10_LITE/CEL_PROJECT {C:/Users/Domenic/Desktop/Quartus_Stuff/CodeGenerated/DE10_LITE/CEL_PROJECT/riscvsingle.sv}

vlog -vlog01compat -work work +incdir+C:/Users/Domenic/Desktop/Quartus_Stuff/CodeGenerated/DE10_LITE/CEL_PROJECT {C:/Users/Domenic/Desktop/Quartus_Stuff/CodeGenerated/DE10_LITE/CEL_PROJECT/my_computer_tb.v}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L fiftyfivenm_ver -L rtl_work -L work -voptargs="+acc"  my_computer_tb

add wave *
view structure
view signals
run -all
