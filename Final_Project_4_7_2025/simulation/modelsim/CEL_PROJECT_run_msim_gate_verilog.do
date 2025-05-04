transcript on
if {[file exists gate_work]} {
	vdel -lib gate_work -all
}
vlib gate_work
vmap work gate_work

vlog -vlog01compat -work work +incdir+. {CEL_PROJECT.vo}

vlog -vlog01compat -work work +incdir+C:/Users/Domenic/Desktop/Quartus_Stuff/CodeGenerated/DE10_LITE/CEL_PROJECT {C:/Users/Domenic/Desktop/Quartus_Stuff/CodeGenerated/DE10_LITE/CEL_PROJECT/my_computer_tb.v}

vsim -t 1ps -L altera_ver -L altera_lnsim_ver -L fiftyfivenm_ver -L gate_work -L work -voptargs="+acc"  my_computer_tb

add wave *
view structure
view signals
run -all
