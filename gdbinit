define connect
target extended-remote localhost:3333
set remote hardware-breakpoint-limit 6
set remote hardware-watchpoint-limit 4
load
monitor reset halt
end

define hook-step
mon cortex_m maskisr on
end

define hookpost-step
mon cortex_m maskisr off
end

define reload
make
monitor reset halt
load
monitor reset halt
end

set trace-commands on
set logging on
set print pretty
