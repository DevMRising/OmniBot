local socket = require("socket")
local json = require("dkjson")
local pack = string.pack

local x, y, z = 1, 1, 1

local data = {{0,0,0}}
local array = {x, y, z}

for i = 1, 5 do
    table.insert(data,array)
end

local row, column = #data , #data[1]
local header = pack("<I4I4>", column, row)

local client = socket.tcp()
client:connect("192.168.3.4", 5698)
             
client:send(column.."x"..row.."\n"..header.."\n"..json.encode(data).."\n")

local response = client:receive()

print("Niga said once: be able to adapt..." .. response)

client:close()