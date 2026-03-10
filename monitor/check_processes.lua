function conky_checkProcesses()
    local processes = {
        { name = "Husky Base", service = "husky_base" },
        { name = "Husky Sensors", service = "sensors" },
        { name = "Husky Localization", service = "localization" },
        { name = "Multiespectral cameras", service = "multiespectral_cameras" },
        { name = "Fisheye cameras", service = "fisheye_cameras" }
    }

    local output = ""
    for _, process in ipairs(processes) do
        local handle = io.popen("systemctl is-active " .. process.service .. ".service 2>/dev/null")
        local status = handle:read("*a"):gsub("%s+", "")
        handle:close()
        local pid = (status == "active") and "1" or ""

        if pid ~= "" then
            output = output .. "${color #FFFFFF}${font Ubuntu:size=9} · " .. process.name .. ":" .. "${color green}${alignr}" .. " Active${color}\n"
        else
            output = output .. "${color #FFFFFF}${font Ubuntu:size=9} · " .. process.name .. ":" .. "${color red}${alignr}" .. " Inactive${color}\n"
        end
    end
    return output
end
