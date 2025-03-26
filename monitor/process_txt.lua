function parseFile(filename)
    -- print("Parsing file: " .. filename)
    local file = io.open(filename, "r")
    if not file then
        return "${color red}Error: File not found${color}\n"
    end

    local output = ""
    for line in file:lines() do
        local key, value = line:match("^(.-):%s*(.+)$") -- Divide la línea en clave y valor
        if key and value then
            -- Determina el color según el valor
            local color = "${color green}" -- Por defecto, verde
            if value:match("N/A") or value:match("nan") or value:match("-1") then
                color = "${color red}"
            end

            -- Formatea la salida con clave en blanco y valor con color dinámico
            output = output .. "${color #FFFFFF}${font Ubuntu:size=9} · " .. key .. ": ${alignr}" .. color .. value .. "${color}\n"
        end
    end

    file:close()
    return output
end


function conky_sensorsStatus()
    local formatted_output = parseFile("/tmp/husky_sensor_status.txt")
    return formatted_output
end

function conky_camerasStatus()
    local formatted_output = parseFile("/tmp/husky_cameras_status.txt")
    return formatted_output
end