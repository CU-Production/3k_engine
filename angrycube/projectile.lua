-- Angry Cube Projectile Script
-- Handles slingshot launching mechanics

local initial_pos = {x = -300, y = 0}
local launched = false
local launch_power = 0
local charge_time = 0
local max_charge = 2.0

return {
    update = function(dt)
        -- Space to charge and launch
        if get_key(32) and not launched then
            charge_time = charge_time + dt
            if charge_time > max_charge then
                charge_time = max_charge
            end
            
            -- Visual feedback
            launch_power = charge_time / max_charge
            if math.floor(charge_time * 4) % 2 == 0 then
                log("Charging... " .. math.floor(launch_power * 100) .. "%")
            end
        end
        
        -- Release to launch
        if not get_key(32) and charge_time > 0 and not launched then
            local power = 800 * launch_power
            -- Launch at 45 degree angle
            local angle = math.pi / 4
            
            log("LAUNCH! Power: " .. math.floor(launch_power * 100) .. "%")
            log("Projectile fired with force " .. math.floor(power))
            
            launched = true
            charge_time = 0
        end
        
        -- R to reset
        if get_key(82) then -- R key
            launched = false
            charge_time = 0
            log("Projectile reset")
        end
    end
}
