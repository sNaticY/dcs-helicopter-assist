-- === DCS -> Python Telemetry Export (UDP) ===
-- 坐标系: 世界坐标 (米) 轴向 x=东, y=海拔, z=北；机体坐标: LoGetVectorVelocity/AngularVelocity 返回机体系

local socket   = require("socket")

-- 配置
local HOST     = "127.0.0.1"
local PORT     = 28777
local INTERVAL = 0.02 -- 50 Hz

-- UDP
local udp      = socket.udp()
udp:settimeout(0)
udp:setpeername(HOST, PORT)

-- 定时
local next_time = 0.0

-- 数值安全
local function safe_json_number(x)
    if x == nil or x ~= x then
        return 0
    end
    return x
end

-- 采样并构造一行 JSON（末尾带换行）
local function get_state()
    local vel                          = LoGetVectorVelocity() -- m/s, 机体坐标
    local accG                         = LoGetAccelerationUnits() -- G,   机体坐标
    local Pitch, Roll, Yaw             = LoGetADIPitchBankYaw() -- rad
    local angRate                      = LoGetAngularVelocity() -- rad/s, 机体坐标
    local sd                           = LoGetSelfData() -- 世界坐标（米）

    local Vx, Vy, Vz                   = 0.0, 0.0, 0.0
    local Ax, Ay, Az                   = 0.0, 0.0, 0.0
    local RollRate, PitchRate, YawRate = 0.0, 0.0, 0.0
    local PosX, PosY, PosZ             = 0.0, 0.0, 0.0

    if vel then
        Vx, Vy, Vz = vel.x, vel.y, vel.z
    end
    if accG then
        local g = 9.80665
        Ax, Ay, Az = accG.x * g, accG.y * g, accG.z * g
    end
    if angRate then
        -- DCS: x=RollRate, y=YawRate, z=PitchRate
        RollRate, YawRate, PitchRate = angRate.x, angRate.y, angRate.z
    end

    if sd and sd.Position then
        -- DCS: Position: x=东, y=海拔, z=北
        PosX, PosY, PosZ = sd.Position.x, sd.Position.y, sd.Position.z
    end

    -- 拼 JSON 行（带换行）
    local payload = string.format(
        '{"Vx":%.6f,"Vy":%.6f,"Vz":%.6f,' ..
        '"Ax":%.6f,"Ay":%.6f,"Az":%.6f,' ..
        '"Pitch":%.4f,"Roll":%.4f,"Yaw":%.4f,' ..
        '"PitchRate":%.4f,"RollRate":%.4f,"YawRate":%.4f,' ..
        '"PosX":%.3f,"PosY":%.3f,"PosZ":%.3f}\n',
        safe_json_number(Vx), safe_json_number(Vy), safe_json_number(Vz),
        safe_json_number(Ax), safe_json_number(Ay), safe_json_number(Az),
        safe_json_number(Pitch), safe_json_number(Roll), safe_json_number(Yaw),
        safe_json_number(PitchRate), safe_json_number(RollRate), safe_json_number(YawRate),
        safe_json_number(PosX), safe_json_number(PosY), safe_json_number(PosZ)
    )
    return payload
end

-- 备份原事件（如有）
local _LuaExportStart          = LuaExportStart
local _LuaExportAfterNextFrame = LuaExportAfterNextFrame
local _LuaExportStop           = LuaExportStop

function LuaExportStart()
    if _LuaExportStart then _LuaExportStart() end
end

function LuaExportAfterNextFrame()
    local t = LoGetModelTime()
    if t and t >= next_time then
        next_time = t + INTERVAL
        local payload = get_state()
        udp:send(payload)
    end
    if _LuaExportAfterNextFrame then _LuaExportAfterNextFrame() end
end

function LuaExportStop()
    if udp then udp:close() end
    if _LuaExportStop then _LuaExportStop() end
end
