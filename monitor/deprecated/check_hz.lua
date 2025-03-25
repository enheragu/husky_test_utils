
-- rostopic hz is continuous function, cannot be used like this :(
-- A timeout can be used, but it slow the whole process of displaying the GUI A LOT
function getTopicHz(topic)
    print("Getting Hz for topic: " .. topic)
    local handle = io.popen("timeout 2 rostopic hz " .. topic .. " 2>&1") -- Detiene el comando después de 5 segundos
    local result = handle:read("*a")
    print("Result: " .. result)
    -- handle:close()

    -- local hz = result:match("average rate:%s*([%d%.]+)") 
    -- print("Hz extracted: " .. tostring(hz))
    -- if hz then
    --     return tonumber(hz)
    -- else
    --     return "N/A"
    -- end
    return "N/A"
end

function handleItems(monitor_items)
    local output = ""
    for _, item in ipairs(monitor_items) do
        -- print("Call hz parser for: " .. item.topic)
        local hz = getTopicHz(item.topic)
        -- print("Topic: " .. item.topic .. ", Hz: " .. tostring(hz))
        if hz == "N/A" then
            output = output .. "${color #FFFFFF}${font Ubuntu:size=9} · " .. item.name .. ":" .. "${alignr}${color red}" .. hz .. "${color}\n"
        else
            output = output .. "${color #FFFFFF}${font Ubuntu:size=9} · " .. item.name .. ":" .. "${alignr}" .. hz .. " Hz\n"
        end
    end
    -- print("Output generated: " .. output)
    return output
end

function conky_camerasHz()
    local monitor_items = {
        {name = "Multiespectral - visible", topic = "/Multiespectral/visible_camera/compressed"},
        {name = "Multiespectral - lwir", topic = "/Multiespectral/lwir_camera/compressed"},
        {name = "Fisheye - frontal", topic = "/Fisheye/frontal_camera/compressed"},
        {name = "Fisheye - rear", topic = "/Fisheye/rear_camera/compressed"}
    }
    return handleItems(monitor_items)    
end

function conky_sensorsHz()
    local monitor_items = {
        {name = "LIDAR", topic = "/ouster/points"},
        {name = "IMU", topic = "/imu/data"},
        {name = "GPS", topic = "/gnss/fix"}
    }
    return handleItems(monitor_items)    
end