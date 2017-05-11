print("Demarrage du vol")
connect()
print("Latitude: "..latitude())
print("Longitude: "..longitude())
ref_lat =-35.363261
ref_long = 149.165230

target_lat = -35.3620491028
target_long = 149.1654357910
target_alt = 20

next_target_lat = -35.3620491028
next_target_long = 149.1654357910
next_target_alt = 100

takeoff()
clearwaypoint()


function cadrillageZone (pointA_lat,pointA_long,pointB_lat,pointB_long,pointC_lat,pointC_long)

  waypoint(pointA_lat,pointA_long,target_alt,3,16)
  waypoint(pointB_lat,pointB_long,target_alt,3,16)
  i = 0
      while(longitude()>pointC_long) do
        i = i + 50
        waypoint(latitude(),longitude()-i,target_alt,3,16)
        waypoint(pointA_lat,longitude(),target_alt,3,16)
        i = i + 50
        waypoint(latitude(),longitude()-i,target_alt,3,16)
        waypoint(pointB_lat,longitude(),target_alt,3,16)


      end
      rtl()
end

--land()
