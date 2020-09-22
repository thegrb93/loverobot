local robot = types.robot
local actuator = types.actuator

local function posAngMatrix(x, y, a)
    local ca, sa = math.cos(a), math.sin(a)
    return matrix{{ca, -sa, x}, {sa, ca, y}, {0, 0, 1}}
end
local function matrixPosAng(m)
    return m[1][3], m[2][3], math.atan2(m[2][1], m[1][1])
end
local function bodyMatrix(body)
    local x, y = body:getPosition()
    local a = body:getAngle()
    return posAngMatrix(x, y, a)
end
local function drawAxis(m)
    local xdir = m * matrix{10, 0, 1}
    local ydir = m * matrix{0, 10, 1}
    love.graphics.setColor(1, 0, 0)
    love.graphics.line(m[1][3], m[2][3], xdir[1][1], xdir[2][1])
    love.graphics.setColor(0, 0.7, 0)
    love.graphics.line(m[1][3], m[2][3], ydir[1][1], ydir[2][1])
    love.graphics.setColor(1, 1, 1)
end

love.physics.setMeter(128)
robot.world = love.physics.newWorld(0, 9.8)
robot.worldbody = love.physics.newBody(robot.world)
robot.worldbodyf = love.physics.newFixture(robot.worldbody, love.physics.newRectangleShape(scrw/2, scrh - 25, scrw, 50))
function robot:initialize()
    self.dt = 1/winmode.refreshrate
    self.frameWorld = posAngMatrix(scrw/2, scrh - 55, 0)
    self.body = robot.worldbody
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
    for k, link in ipairs(self.links) do
        link:calcWorld()
    end
    for i, link in ipairs(self.links) do
        link:createBody()
    end
end

function robot:compute()
    for k, link in ipairs(self.links) do
        link:measureJoint()
    end
    for k, link in ipairs(self.links) do
        link:calcWorld()
    end
    for k, link in ipairs(self.links) do
        link:jointControl()
    end
end

function robot:render()
    self:compute()
    robot.world:update(self.dt)
    
    love.graphics.rectangle("fill", 0, scrh - 50, scrw, 50)
    for k, link in ipairs(self.links) do
        link:draw()
    end
end

function actuator:initialize(parent, frame, shape)
    self.parent = parent
    self.frame = frame
    self.shape = shape
    self.theta = 0
    self.dtheta = 0
end

function actuator:createBody()
    local x, y, a = matrixPosAng(self.frameWorld)
    self.body = love.physics.newBody(robot.world, x, y, "dynamic")
    self.body:setAngle(a)
    self.bodyf = love.physics.newFixture(self.body, self.shape)
    self.joint = love.physics.newRevoluteJoint(self.parent.body, self.body, x, y, false)
end

function actuator:draw()
    love.graphics.polygon("fill", self.body:getWorldPoints(self.shape:getPoints()))
    drawAxis(self.frameWorld)
    self.body:applyForce(-0.5,0)
end

function actuator:applyTorque(t)
    self.parent.body:applyTorque(-t)
    self.body:applyTorque(t)
end

function actuator:measureJoint()
    self.theta, self.dtheta = self.joint:getJointAngle(), self.joint:getJointSpeed()
end

function actuator:jointControl()
    local perr = (0 - self.theta) * 200
    local derr = (0 - self.dtheta) * 20
    self:applyTorque(perr + derr)
end

function actuator:calcWorld()
    self.frameWorld = self.parent.frameWorld * self.frame * posAngMatrix(0, 0, self.theta)
end

robot:new()
