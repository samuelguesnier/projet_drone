
dofile("LuaXML-master/xml.lua")
dofile("LuaXML-master/handler.lua")


verificationXML = require("verificationXML")



print("Demarrage du vol")
connect()

print("Latitude: "..latitude())
print("Longitude: "..longitude())




verificationXML.cadrillageZone()
