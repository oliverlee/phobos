define connect
target extended-remote localhost:3333
set remote hardware-breakpoint-limit 6
set remote hardware-watchpoint-limit 4
load
mon reset halt
end

define hook-step
mon cortex_m maskisr on
end

define hookpost-step
mon cortex_m maskisr off
end

