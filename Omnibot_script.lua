local socket = require("socket")
local json = require("dkjson")

-- Generating vector table
local vectorList = {}

function sysCall_init()
    sim = require('sim')

    -- Wheel Joints
    Base = sim.getObjectHandle('/Base_dyn')
    wheelJoint_FL = sim.getObjectHandle('/rollingJoint_fl')
    wheelJoint_FR = sim.getObjectHandle('/rollingJoint_fr')
    wheelJoint_RL = sim.getObjectHandle('/rollingJoint_rl')
    wheelJoint_RR = sim.getObjectHandle('/rollingJoint_rr')

    -- Path data
    path = sim.getObjectHandle('/Path2')
    local pathData = sim.unpackDoubleTable(sim.readCustomDataBlock(path, 'PATH'))
    local nRows = #pathData // 7
    pathPositions = {}
    for i = 0, nRows - 1 do
        local base = i * 7
        pathPositions[3 * i + 1] = pathData[base + 1]
        pathPositions[3 * i + 2] = pathData[base + 2]
        pathPositions[3 * i + 3] = pathData[base + 3]
    end
    pathLengths, totalLength = sim.getPathLengths(pathPositions, 3)

    -- Movement parameters
    velocity = 0.2                -- linear speed (m/s)
    vectorT = 0.4                 -- duration of each command (sec)
    lookaheadDistance = 0.1       -- slightly reduced to match lower speed (m/s)

    -- Height offset
    floorHeight = 0.0
    local initialPos = sim.getObjectPosition(Base, -1)
    baseHeightOffset = initialPos[3] - floorHeight

    wheelRadius = 0.0562
    wheelCircumference = 2 * math.pi * wheelRadius
    wheelRotationAngle = 0

    posAlongPath = 0
    previousSimulationTime = sim.getSimulationTime()
    previousTargetAngle = nil

    sim.setObjectOrientation(Base, -1, {0, 0, 0})
    sim.setStepping(true)
end

-- Normalize angle
local function normalizeAngle(rad)
    local twoPi = 2 * math.pi
    rad = rad % twoPi
    if rad < 0 then rad = rad + twoPi end
    return rad
end

-- Path start point
local function getPointAtDistance(dist)
    dist = dist % totalLength
    return sim.getPathInterpolatedConfig(pathPositions, pathLengths, dist)
end

function sysCall_thread()
    -- Initialisation: starting point on path
    local currentPathPoint = getPointAtDistance(posAlongPath)
    -- Compute lookahead point for initial angle
    local lookaheadDist = math.min(lookaheadDistance, totalLength)
    local sLook = posAlongPath + lookaheadDist
    if sLook > totalLength then sLook = sLook - totalLength end
    local lookaheadPoint = getPointAtDistance(sLook)
    local dx = lookaheadPoint[1] - currentPathPoint[1]
    local dy = lookaheadPoint[2] - currentPathPoint[2]
    local initialAngle = math.atan2(dy, dx)
    initialAngle = normalizeAngle(initialAngle)
    previousTargetAngle = initialAngle

    local omega = 0.0 -- omega is always 0.0
    
    local firstVec = {velocity, initialAngle, omega, vectorT}
    table.insert(vectorList, firstVec)
    print(string.format("{%.2f, %.4f, %.4f, %.1f}", velocity, initialAngle, omega, vectorT))

    local lastVectorTime = 0.0

    while not sim.getSimulationStopping() do
        local t = sim.getSimulationTime()
        local dt = t - previousSimulationTime
        if dt <= 0 then dt = sim.getSimulationTimeStep() end

        -- Advance along the path (kinematic)
        posAlongPath = (posAlongPath + velocity * dt) % totalLength
        local currentPosOnPath = getPointAtDistance(posAlongPath)
        currentPosOnPath[3] = floorHeight + baseHeightOffset
        sim.setObjectPosition(Base, -1, currentPosOnPath)

        -- Generate new command every vectorT
        if t - lastVectorTime >= vectorT then
            -- Compute lookahead point along the path
            local sLook = posAlongPath + lookaheadDistance
            if sLook > totalLength then sLook = sLook - totalLength end
            local lookaheadPos = getPointAtDistance(sLook)
            local currentPosXY = {currentPosOnPath[1], currentPosOnPath[2]}

            -- Desired direction angle to lookahead point
            local dx = lookaheadPos[1] - currentPosXY[1]
            local dy = lookaheadPos[2] - currentPosXY[2]
            local desiredAngle = math.atan2(dy, dx)
            desiredAngle = normalizeAngle(desiredAngle)

            local vec = {velocity, desiredAngle, omega, vectorT}
            table.insert(vectorList, vec)
            print(string.format("{%.2f, %.2f, %.1f, %.1f}", velocity, desiredAngle, 0.0, vectorT))

            previousTargetAngle = desiredAngle
            lastVectorTime = lastVectorTime + vectorT
        end

        local wheelRotationSpeed = velocity / wheelCircumference
        wheelRotationAngle = wheelRotationAngle + wheelRotationSpeed * 2 * math.pi * dt
        sim.setJointPosition(wheelJoint_FL, wheelRotationAngle)
        sim.setJointPosition(wheelJoint_FR, wheelRotationAngle)
        sim.setJointPosition(wheelJoint_RL, wheelRotationAngle)
        sim.setJointPosition(wheelJoint_RR, wheelRotationAngle)

        previousSimulationTime = t
        sim.step()
    end
end

function sysCall_cleanup()
    sim.setStepping(false)

    -- Send all vectors via TCP
    if #vectorList > 0 then
        local client = socket.tcp()
        client:settimeout(5)
        local success, err = client:connect("192.168.3.4", 6767)
        if not success then
            print("TCP connection failed: " .. tostring(err))
            client:close()
            return
        end

        local encoded = json.encode(vectorList)
        client:send(encoded)

        local response = client:receive()
        if response then
            print("Server response: " .. response)
        end
        
        client:close()
        print("Sent " .. #vectorList .. " vectors via TCP")
    else
        print("No vectors to send")
    end
end