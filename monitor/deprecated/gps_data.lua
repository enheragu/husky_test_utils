function getGpsParams(topic)
    local handle = io.popen("rostopic echo -n 1 " .. topic .. " 2>&1")
    local result = handle:read("*a")
    handle:close()

    local mode_gps = result:match("status:%s*status:%s*(%d+)") -- Search for status
    local cov_0 = result:match("position_covariance:%s*%[(%d+%.?%d*)") -- Parse relevant covariance values cov[0] and cov[4]
    local cov_4 = result:match("position_covariance:%s*[%d%.]+,%s*[%d%.]+,%s*[%d%.]+,%s*[%d%.]+,%s*(%d+%.?%d*)")

    mode_gps = mode_gps or "N/A"
    cov_0 = cov_0 or "N/A"
    cov_4 = cov_4 or "N/A"

    return mode_gps, cov_0, cov_4
end

function mapGpsMode(mode)
    local gps_modes = {
        ["-1"] = "No Fix",
        ["0"] = "Fix",
        ["1"] = "SBAS Fix",
        ["2"] = "GBAS Fix"
    }

    return gps_modes[mode] or "Unknown"
end

function conky_gpsStatus()
    local gps_topic = "/gnss/fix"

    local mode_gps, cov_0, cov_4 = getGpsParams(gps_topic)
    local mapped_mode_gps = (mode_gps == "N/A" and "N/A" or mapGpsMode(mode_gps))

    local output = "${color #FFFFFF}${font Ubuntu:size=9} · Mode GPS: ${alignr}" .. (mapped_mode_gps == "N/A" and "${color red}" or "") .. mapped_mode_gps .. "${color}\n"
    output = output .. "${color #FFFFFF}${font Ubuntu:size=9} · Cov GPS: ${alignr}[" .. (cov_0 == "N/A" and "${color red}" or "") .. cov_0 .. "${color}, " .. (cov_4 == "N/A" and "${color red}" or "") .. cov_4 .. "${color}]"

    return output
end
