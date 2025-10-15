def linear_map_constrain_int(value, from_low, from_high, to_low, to_high):
        
        factor = (value - from_low)/(from_high - from_low) # return 0-1 float

        mapped_val = (to_high - to_low) * factor + to_low # Scale factor by range of output and add lower offset

        return round(min((to_high, max((to_low, mapped_val)) )) )





range_start = 128
range_max = 255
range_midpoint = 192




speed = 100

# default case is motor stop
send_val = range_midpoint

# Backwards
if speed < 0:
    send_val = linear_map_constrain_int(100+speed, 0, 100, range_start, range_midpoint)

#Forwards
elif speed > 0:
    send_val = linear_map_constrain_int(speed, 0, 100, range_midpoint, range_max)


print(send_val)