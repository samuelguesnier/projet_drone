dofile("LuaXML-master/xml.lua")
dofile("LuaXML-master/handler.lua")

local filename = "waypoints.gpx"
local xmltext = ""
local f, e = io.open(filename, "r")
if f then
  --Gets the entire file content and stores into a string
  xmltext = f:read("*a")
else
  error(e)
end

--Instantiate the object the states the XML file as a Lua table
local xmlhandler = simpleTreeHandler()

--Instantiate the object that parses the XML to a Lua table
local xmlparser = xmlParser(xmlhandler)
xmlparser:parse(xmltext)

function test()

  print(xmlhandler.root.gpx.wpt[2]._attr.lat)

end

function round(num, numDecimalPlaces)
  local mult = 10^(numDecimalPlaces or 0)
  return math.floor(num * mult + 0.5) / mult
end

print(round(149.16375732422,5))
