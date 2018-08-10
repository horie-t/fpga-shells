set group_mem [get_clocks -quiet {clk_pll_i}]
set group_sys [get_clocks -quiet {sys_clk_pin                    \
                                  clk_out*_nexys4ddr_sys_clock_mmcm1  \
                                  clk_out*_nexys4ddr_sys_clock_mmcm2}]

puts "group_mem: $group_mem"
puts "group_sys: $group_sys"

set groups [list]
if { [llength $group_mem]    > 0 } { lappend groups -group $group_mem }
if { [llength $group_sys]    > 0 } { lappend groups -group $group_sys }

puts "set_clock_groups -asynchronous $groups"
set_clock_groups -asynchronous {*}$groups
