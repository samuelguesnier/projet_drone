local verificationXML = {}

dofile("LuaXML-master/xml.lua")
dofile("LuaXML-master/handler.lua")

--xmlparser
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


--Fonction haverline pour les distances
local function distance(x1, y1, x2, y2)
r=0.017453292519943295769236907684886127;
x1= x1*r; x2= x2*r; y1= y1*r; y2= y2*r; dy = y2-y1; dx = x2-x1;
a = math.pow(math.sin(dx/2),2) + math.cos(x1) * math.cos(x2) * math.pow(math.sin(dy/2),2); c = 2 * math.asin(math.sqrt(a)); d = 6372.8 * c;
return d;
end

--Parcous simplement les points du fichier
  function verificationXML.parcours()
      clearwaypoint()
      takeoff()


    for k, p in pairs(xmlhandler.root.gpx.wpt) do

      local altitude = p.ele + 20
      local Latitude = p._attr.lat
      local Longitude = p._attr.lon
      local nom = p.name

      local distance = distance(longitude(),latitude(),Longitude,Latitude)
      local distance = distance * 1000

      print("La distance entre les deux pts est de: "..distance.." metres")
      print("Nom: ", nom, "Altitude:", altitude, "Latitude: ", Latitude, "Longitude: ", Longitude)
      if (distance<1000) then
        waypoint(Latitude,Longitude,altitude,3,16)


      else
        print("La distance entre les deux points est trop elevee "..distance)
      end

     end
     rtl()
  end

 function verificationXML.cadrillageZone()
    local indice = 0
    for k, p in pairs(xmlhandler.root.gpx.wpt) do
      indice = indice + 1
    end

    if (indice == 3)   then

    local pointA_lat = xmlhandler.root.gpx.wpt[1]._attr.lat

    local pointA_long = xmlhandler.root.gpx.wpt[1]._attr.lon
    local pointA_alt = xmlhandler.root.gpx.wpt[1].ele


    local pointB_lat = xmlhandler.root.gpx.wpt[2]._attr.lat
    local pointB_alt = xmlhandler.root.gpx.wpt[2].ele

    local pointC_lat = xmlhandler.root.gpx.wpt[3]._attr.lat
    local pointC_long = xmlhandler.root.gpx.wpt[3]._attr.lon
    local pointC_alt = xmlhandler.root.gpx.wpt[3].ele

    clearwaypoint()
    takeoff()
      waypoint(pointA_lat,pointA_long,20,3,16)

      waypoint(pointB_lat,pointA_long,20,3,16)
      local pas = 0.0005
  

      local longitudeActuelle = longitude()


        while (longitudeActuelle < tonumber(pointC_long)) do
            waypoint(latitude(),longitude()-pas,20,3,16)
            waypoint(pointA_lat,longitude(),20,3,16)

            waypoint(latitude(),longitude()-pas,20,3,16)
            waypoint(pointB_lat,longitude(),20,3,16)
          end
      rtl()
  end
end




return verificationXML
