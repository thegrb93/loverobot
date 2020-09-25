local robot = types.robot
local actuator = types.actuator

local function posAngMatrix(x, y, a)
    local ca, sa = math.cos(a), math.sin(a)
    return matrix{{ca, -sa, 0, x}, {sa, ca, 0, y}, {0, 0, 1, 0}, {0, 0, 0, 1}}
end
local function matrixPosAng(m)
    return m[1][4], m[2][4], math.atan2(m[2][1], m[1][1])
end
local function invertHMatrix(m)
    local m2 = m^"T"
    m2[4][1] = 0
    m2[4][2] = 0
    m2[4][3] = 0
    local t = matrix{m[1][4], m[2][4], m[3][4], 0}
    t = m2 * t
    m2[1][4] = -t[1][1]
    m2[2][4] = -t[2][1]
    m2[3][4] = -t[3][1]
    return m2
end
local function offsetInertia(I, mass, pos)
    pos = matrix{pos[1][1], pos[2][1], pos[3][1], 0}
    local posDot = pos[1][1]^2 + pos[2][1]^2 + pos[3][1]^2
    local m = matrix{{posDot, 0, 0, 0}, {0, posDot, 0, 0}, {0, 0, posDot, 0}, {0, 0, 0, 0}}
    return I + mass*(matrix{{posDot, 0, 0, 0}, {0, posDot, 0, 0}, {0, 0, posDot, 0}, {0, 0, 0, 0}} - pos*pos^"T")
end
local function bodyMatrix(body)
    local x, y = body:getPosition()
    local a = body:getAngle()
    return posAngMatrix(x, y, a)
end
local function drawAxis(m)
    local xdir = m * matrix{10, 0, 0, 1}
    local ydir = m * matrix{0, 10, 0, 1}
    love.graphics.setColor(1, 0, 0)
    love.graphics.line(m[1][4], m[2][4], xdir[1][1], xdir[2][1])
    love.graphics.setColor(0, 0.7, 0)
    love.graphics.line(m[1][4], m[2][4], ydir[1][1], ydir[2][1])
    love.graphics.setColor(1, 1, 1)
end

love.physics.setMeter(128)
local G = 9.8
robot.world = love.physics.newWorld(0, G)
robot.worldbody = love.physics.newBody(robot.world)
robot.worldbodyf = love.physics.newFixture(robot.worldbody, love.physics.newRectangleShape(scrw/2, scrh - 25, scrw, 50))
robot.dt = 1/winmode.refreshrate
function robot:initialize()
    self.jointFrame = posAngMatrix(0, 0, 0)
    self.frameWorld = posAngMatrix(scrw/2, scrh - 55, 0)
    self.body = robot.worldbody

    self.vel = matrix{0,0,0,0}
    self.angvel = matrix{0,0,0,0}
    self.accel = matrix{0,-G,0,0}
    self.angaccel = matrix{0,0,0,0}

    self:buildActuators()
    hook.add("render", self)
end

function robot:buildActuators()
    local jointChain = {
        {posAngMatrix(0, 0, 0), love.physics.newRectangleShape(0, -25, 5, 50)},
        {posAngMatrix(0, -50, 0), love.physics.newRectangleShape(0, -25, 5, 50)},
        {posAngMatrix(0, -50, 0), love.physics.newRectangleShape(0, -25, 5, 50)},
        {posAngMatrix(0, -50, 0), love.physics.newRectangleShape(0, -25, 5, 50)},
    }

    self.links = {}
    for i, chain in ipairs(jointChain) do
        self.links[i] = actuator:new(self.links[i-1] or self, chain[1], chain[2])
    end
end

function robot:compute()
    for k, link in ipairs(self.links) do
        link:measureJoint()
    end
    for k, link in ipairs(self.links) do
        link:calcForward()
    end
    self.links[#self.links].f = self.links[#self.links].F
    self.links[#self.links].n = self.links[#self.links].N
    for i=#self.links, 2, -1 do
        local link = self.links[i]
        link:calcBackward()
    end
    for k, link in ipairs(self.links) do
        link:jointControl()
    end
end

function robot:render()
    self:compute()
    robot.world:update(robot.dt)
    
    love.graphics.rectangle("fill", 0, scrh - 50, scrw, 50)
    for k, link in ipairs(self.links) do
        link:draw()
    end
end

function actuator:initialize(parent, frame, shape)
    self.parent = parent
    self.frame = frame
    self.jointFrame = frame
    self.shape = shape
    self.ltheta = 0

    self.theta = posAngMatrix(0, 0, 0)
    self.dtheta = matrix{0,0,0,0}

    self.targetDTheta = matrix{0,0,0,0}
    self.targetDDTheta = matrix{0,0,0,0}
    
    self.f = matrix{0,0,0,0}
    self.n = matrix{0,0,0,0}

    self:createBody()
end

function actuator:createBody()
    self.frameWorld = self.parent.frameWorld * self.frame
    local frameWorldInv = invertHMatrix(self.frameWorld)

    local x, y, a = matrixPosAng(self.frameWorld)
    self.body = love.physics.newBody(robot.world, x, y, "dynamic")
    self.body:setAngle(a)
    self.bodyf = love.physics.newFixture(self.body, self.shape)
    self.joint = love.physics.newRevoluteJoint(self.parent.body, self.body, x, y, false)
    local comX, comY = self.body:getWorldCenter()
    self.mass = self.body:getMass()
    self.centerOfMass = frameWorldInv * matrix{comX, comY, 0, 1}
    local I = matrix{{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, self.body:getInertia(), 0},{0, 0, 0, 0}}
    self.inertia = offsetInertia(I, self.mass, self.centerOfMass)
end

function actuator:draw()
    love.graphics.polygon("fill", self.body:getWorldPoints(self.shape:getPoints()))
    drawAxis(self.frameWorld)
end

function actuator:applyTorque(t)
    self.parent.body:applyTorque(-t)
    self.body:applyTorque(t)
end

function actuator:measureJoint()
    local theta, dtheta = self.joint:getJointAngle(), self.joint:getJointSpeed()
    local ddtheta = (dtheta - self.ltheta) / robot.dt
    self.ltheta = dtheta
    self.theta = posAngMatrix(0, 0, theta)
    self.dtheta = matrix{0,0,dtheta,0}

    self.targetDTheta = matrix{0,0,dtheta-theta*0,0}
    self.targetDDTheta = matrix{0,0,ddtheta,0}

    self.jointFrame = self.frame * self.theta
    self.jointFrameInv = invertHMatrix(self.jointFrame)
    self.frameWorld = self.parent.frameWorld * self.jointFrame
end

function actuator:jointControl()
    local perr = (0 - self.theta[3][1]) * 200
    local derr = (0 - self.dtheta[3][1]) * 20
    self:applyTorque(self.n[3][1]*0.1)
    self.body:applyForce(-2,0)
end

function actuator:calcForward()
    local pos = self.parent.jointFrame * matrix{self.frame[1][4], self.frame[2][4], 0, 0}

    self.angvel = self.jointFrame * self.parent.angvel + self.targetDTheta
    self.angaccel = self.jointFrame * self.parent.angaccel + (self.jointFrame * self.parent.angvel):cross(self.targetDTheta) + self.targetDDTheta

    self.vel = self.jointFrame * (self.parent.angaccel:cross(pos) + self.parent.angvel:cross(self.parent.angvel:cross(pos)) + self.parent.vel)
    self.velC = self.angaccel:cross(self.centerOfMass) + self.angvel:cross(self.angvel:cross(self.centerOfMass)) + self.vel

    self.F = self.mass * self.velC
    self.N = self.inertia * self.angaccel + self.angvel:cross(self.inertia * self.angvel)
end

function actuator:calcBackward()
    local pos = self.parent.jointFrame * matrix{self.frame[1][4], self.frame[2][4], 0, 0}
    self.parent.f = self.jointFrameInv * self.f + self.parent.F
    self.parent.n = self.jointFrameInv * self.n + self.parent.centerOfMass:cross(self.parent.F) + pos:cross(self.jointFrameInv*self.f) + self.parent.N
end

robot:new()
