uint32[] part_id

float32[] part_location  # Format is (x,y,yaw) in metres and radians respectively and then repeated as has to be 1D array
                           # location_array_2D=np.reshape(loc_msg,(int(len(loc_array)/3),3)) to repack into 2D array where each row is for a new ID

