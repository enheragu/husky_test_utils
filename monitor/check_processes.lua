function conky_checkProcesses()
    local processes = {
        { name = "Husky Base", command = "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch husky_base base.launch" },
        { name = "Husky Sensors", command = "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch husky_manager sensors_manager.launch" },
        { name = "Multiespectral cameras", command = "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch multiespectral_fb multiespectral.launch" },
        { name = "Fisheye cameras", command = "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch husky_manager fisheye_cameras.launch" }
    }

    local output = ""
    for _, process in ipairs(processes) do
        local handle = io.popen("pgrep -f '^" .. process.command .. "$'")
        local pid = handle:read("*a"):gsub("%s+", "")
        -- print("Cmd search: [" .. pid .. "] for command [" .. command .. "]")
        -- print(name .. ": " .. "pgrep -f '" .. command .. "'" .. " -> " .. pid)
        handle:close()

        if pid ~= "" then
            output = output .. "${color #FFFFFF}${font Ubuntu:size=9} · " .. process.name .. ":" .. "${color green}${alignr}" .. " Active${color}\n"
        else
            output = output .. "${color #FFFFFF}${font Ubuntu:size=9} · " .. process.name .. ":" .. "${color red}${alignr}" .. " Inactive${color}\n"
        end
    end
    return output
end
