
print("Demarrage du vol")
connect()
print("Latitude: "..latitude())
print("Longitude: "..longitude())





takeoff()
clearwaypoint()


function cadrillageZone (pointA_lat,pointA_long,pointB_lat,pointC_long,pas,alt)

  waypoint(pointA_lat,pointA_long,target_alt,3,16)

  waypoint(pointB_lat,pointA_long,target_alt,3,16)



      while(longitude()>pointC_long) do
        waypoint(latitude(),longitude()-pas,target_alt,3,16)
        waypoint(pointA_lat,longitude(),target_alt,3,16)

        waypoint(latitude(),longitude()-pas,target_alt,3,16)
        waypoint(pointB_lat,longitude(),target_alt,3,16)
      end
      rtl()
end

cadrillageZone(-35.363317,149.164299,-35.362149,149.161559,0.0005)
