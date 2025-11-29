-- Example Lua Script for 2D Engine
-- This script demonstrates basic entity behavior

local time = 0

return {
    update = function(dt)
        time = time + dt
        
        -- Example: Print to console every second
        if math.floor(time) > math.floor(time - dt) then
            log("Script running... time = " .. math.floor(time))
        end
        
        -- Example: Check input
        if get_key(32) then -- Space key (SAPP_KEYCODE_SPACE = 32)
            log("Space key pressed!")
        end
        
        -- You can add game logic here:
        -- - Move entities based on input
        -- - Spawn particles
        -- - Handle collision responses
        -- - Update AI behavior
    end
}
