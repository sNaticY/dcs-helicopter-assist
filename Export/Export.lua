-- === DCS -> Python Telemetry Export (UDP) ===
local socket = require("socket")
local host, port = "127.0.0.1", 28777
local udp = socket.udp()
udp:settimeout(0)
udp:setpeername(host, port)

local next_time = 0
local interval = 0.02  -- 50 Hz

local function safe_json_number(x)
    if x == nil or x ~= x then return 0 end
    return x
end

local function get_state()
    local vel = LoGetVectorVelocity()           -- m/s {x, y, z} 机体坐标
    local acc = LoGetAccelerationUnits()        -- G 单位 {x, y, z}
    local Pitch, Roll, Yaw = LoGetADIPitchBankYaw()
    local angRate = LoGetAngularVelocity()          -- rad/s {x, y, z} 机体坐标

    local Vx, Vy, Vz = 0,0,0
    local Ax, Ay, Az = 0,0,0
    local RollRate, PitchRate, YawRate = 0,0,0

    if vel then Vx, Vy, Vz = vel.x, vel.y, vel.z end
    if acc then Ax, Ay, Az = acc.x*9.80665, acc.y*9.80665, acc.z*9.80665 end -- 转成 m/s^2
    if angRate then RollRate, YawRate, PitchRate = angRate.x, angRate.y, angRate.z end

    -- 统一发 JSON 行，Python 端按行读
    local payload = string.format(
        '{"Vx":%.6f,"Vy":%.6f,"Vz":%.6f,"Ax":%.6f,"Ay":%.6f,"Az":%.6f,"Pitch":%.4f,"Roll":%.4f,"Yaw":%.4f,"PitchRate":%.4f,"RollRate":%.4f,"YawRate":%.4f}\n',
        safe_json_number(Vx), safe_json_number(Vy), safe_json_number(Vz),
        safe_json_number(Ax), safe_json_number(Ay), safe_json_number(Az),
        safe_json_number(Pitch), safe_json_number(Roll), safe_json_number(Yaw),
        safe_json_number(PitchRate), safe_json_number(RollRate), safe_json_number(YawRate)
    )
    return payload
end

-- 备份原事件（如有）
local _LuaExportStart        = LuaExportStart
local _LuaExportStop         = LuaExportStop
local _LuaExportAfterNextFrame = LuaExportAfterNextFrame

function LuaExportStart()
    if _LuaExportStart then _LuaExportStart() end
end

function LuaExportAfterNextFrame()
    local t = LoGetModelTime()
    if t and t >= next_time then
        next_time = t + interval
        local payload = get_state()
        udp:send(payload)
    end
    if _LuaExportAfterNextFrame then _LuaExportAfterNextFrame() end
end

function LuaExportStop()
    if udp then udp:close() end
    if _LuaExportStop then _LuaExportStop() end
end
