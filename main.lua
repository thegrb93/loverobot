scrw, scrh, winmode = love.window.getMode()
class = require("lib/middleclass")
hook = require("lib/hook")
matrix = require("lib/matrix")

types = {
    robot = class("robot"),
    actuator = class("actuator"),
}

require("robot")

function love.run()
    -- love.load(love.arg.parseGameArguments(arg), arg)
    hook.call("postload")

    -- Main loop time.
    return function()
        -- Process events.
        love.event.pump()
        for name,a,b,c,d,e,f in love.event.poll() do
            if name == "quit" then
                return a or 0
            end
            hook.call(name,a,b,c,d,e,f)
        end

        love.graphics.origin()
        love.graphics.clear(love.graphics.getBackgroundColor())
        hook.call("render")
        love.graphics.present()
    end
end
