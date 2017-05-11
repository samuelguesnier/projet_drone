#!/usr/bin/env lua
---Sample application to read a XML file and print it on the terminal.
--@author Manoel Campos da Silva Filho - http://manoelcampos.com

dofile("../xml.lua")
dofile("../handler.lua")

---Recursivelly prints a table
--@param tb The table to be printed
--@param level Only internally used to indent the print.


local filename = "waypoints.xml"
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

--Recursivelly prints the table
--printable(xmlhandler.root)

--Manually prints the table (once that the XML structure for this example is previously known)
for k, p in pairs(xmlhandler.root.gpx.wpt) do
  print("Altitude:", p.ele)
end
