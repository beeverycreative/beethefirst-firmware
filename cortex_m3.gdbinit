define hook-stop
monitor cortex_m3 maskisr on
end
define hookpost-stop
monitor cortex_m3 maskisr off
end

define hook-step
monitor cortex_m3 maskisr on
end
define hookpost-step
monitor cortex_m3 maskisr off
end
