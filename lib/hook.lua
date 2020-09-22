local class = require("lib/middleclass")
local hook = {}
local hooktbl = class("hooktbl")
local hooktbloftype = class("hooktbloftype", hooktbl)

function hooktbl:initialize()
    self.nhooks = 0
end

function hooktbl:find(id)
    local pnode
    local node = self.hooks
    for i=1, self.nhooks do
        if node.id == id then return pnode, node end
        pnode = node
        node = node.next
    end
end

function hooktbl:add(id, func)
    local pnode, node = self:find(id)
    if node then
        node.func = func
    else
        local node = {
            id = id,
            func = func,
            next = self.hooks
        }
        self.hooks = node
        self.nhooks = self.nhooks + 1
    end
end

function hooktbl:remove(id)
    local pnode, node = self:find(id)
    if node then
        if pnode then
            pnode.next = node.next
        else
            self.hooks = node.next
        end
        self.nhooks = self.nhooks - 1
    end
end

function hooktbl:call(...)
    local node = self.hooks
    for i=1, self.nhooks do
        node.func(...)
        node = node.next
    end
end

function hooktbloftype:call(...)
    local node = self.hooks
    for i=1, self.nhooks do
        node.func(node.id, ...)
        node = node.next
    end
end

local hooktbls = setmetatable({},{__index=function(self,k) local t=hooktbl:new() self[k]=t return t end})
local hooktbloftypes = setmetatable({},{__index=function(self,k) local t=hooktbloftype:new() self[k]=t return t end})

function hook.add(name, id, func)
    if type(id)=="table" then
        hooktbloftypes[name]:add(id, id[name])
    else
        hooktbls[name]:add(id, func)
    end
end

function hook.remove(name, id)
    if type(id)=="table" then
        hooktbloftypes[name]:remove(id)
    else
        hooktbls[name]:remove(id)
    end
end

function hook.call(name, ...)
    hooktbloftypes[name]:call(...)
    hooktbls[name]:call(...)
end

return hook
