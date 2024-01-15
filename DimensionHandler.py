import math
import FileHandler

params_with_units = FileHandler.csv_to_map("Parameters.csv")
params = dict()
for name, (val, unit) in params_with_units.items():
    params[name] = val

## CONSTANTS ##

BOLT_SPACING_FACTOR = 0.5
STEERING_MOUNT_FILLET_RADIUS_FACTOR = 0.25

## VALIDATION ##

def validate_rover_width():
    target_frame_to_wheel_width = (params["rover_width"] - params["frame_width"]) / 2
    frame_to_wheel_width = sum([
        params["upper_shaft_frame_clearance"],
        params["swingarm_thickness"],
        2 * params["linkage_thickness"],
        2 * params["middle_wheel_clearance"],
        params["upper_shaft_overhang"],
        params["middle_wheel_shaft_length"],
        params["wheel_thickness"]
    ])
    
    return (frame_to_wheel_width <= target_frame_to_wheel_width, frame_to_wheel_width)

## HELPERS ##

# Takes "upper_" or "lower_" pivot prefix and returns housing diameter
def get_pivot_housing_diameter(prefix):
    return params[prefix + "bearing_diameter"] + 2 * (params["pivot_housing_bolt_diameter"] + 2 * params["pivot_housing_min_wall_thickness"])

# Returns linkage angle (in degrees) and extended length
def get_linkage_angle_and_extended_length(height, width):
    return (math.degrees(math.atan2(height, width)), math.sqrt(height**2 + width**2))

# Constructs and returns linkage map
def get_linkage_map(length, angle):
    return {
        "linkage_thickness": (params["linkage_thickness"], "mm"),
        "width": (params["linkage_width"], "mm"),
        "wall_thickness": (params["linkage_wall_thickness"], "mm"),
        "length": (length, "mm"),
        "angle": (angle, None),
        "bolt_diameter": (params["linkage_mount_bolt_diameter"], "mm"),
        "bolt_spacing": (BOLT_SPACING_FACTOR * params["linkage_mount_bolt_diameter"], "mm")
    }

# Constructs and returns partial shaft map
def get_shaft_map(prefix):
    return {
        "shaft_diameter": (params[prefix + "shaft_diameter"], "mm"),
        "ret_ring_inner_diameter": (params[prefix + "ret_ring_inner_diameter"], "mm"),
        "ret_ring_thickness": (params[prefix + "ret_ring_thickness"], "mm")
    }

# Appends key-value pairs from one map to another based on provided key list
def append_map(key_list, src, dest):
    dest.update({key: src[key] for key in key_list if key in src})
    return

### All 'update' functions update respective text files and return maps ###

## LINKAGES ##

# Returns offset in addition to linkage map
def update_front_rocker_linkage():
    # Compute length and angle
    height = ( params["ground_clearance"] + (0.5 * params["frame_height"]) ) - ( params["corner_wheel_asm_height"] + params["steering_asm_height"] + params["front_steering_mount_neck_height"]) # h1 - (h2 + n_y)
    width = 0.5 * (params["rover_length"] - params["wheel_diameter"]) # w 
    angle, extended_length = get_linkage_angle_and_extended_length(height, width) #theta, l
    alpha_rad = math.radians((angle + 90) / 2) # alpha
    upper_pivot_housing_radius = get_pivot_housing_diameter("upper_") / 2 # R
    offset = params["linkage_width"] / (2 * math.tan(alpha_rad)) # d
    length = extended_length - (upper_pivot_housing_radius + 2 * params["linkage_mount_base_length"] + offset) # l - (R + 2b + d)
    
    # Update linkage file and return map
    linkage = get_linkage_map(length, angle)
    FileHandler.map_to_text_file(linkage, "front_rocker_linkage.txt")
    return linkage, offset

def update_rear_rocker_linkage():
    # Compute length and angle
    height = ( ( params["ground_clearance"] + (0.5 * params["frame_height"]) ) - 
        ( params["corner_wheel_asm_height"] + params["steering_asm_height"] + params["rear_steering_mount_neck_height"] + (params["linkage_width"] / 2) ) ) # h1 - h2 - (n_y + t / 2)
    width = params["rover_length"] / 4 # w
    angle, extended_length = get_linkage_angle_and_extended_length(height, width) #theta, l
    upper_pivot_housing_radius = get_pivot_housing_diameter("upper_") / 2 # R
    lower_pivot_housing_radius = get_pivot_housing_diameter("lower_") / 2 # r
    length = extended_length - (upper_pivot_housing_radius + lower_pivot_housing_radius + (2 * params["linkage_mount_base_length"])) # l - (R + r + 2b)

    # Update linkage file and return map
    linkage = get_linkage_map(length, angle)
    FileHandler.map_to_text_file(linkage, "rear_rocker_linkage.txt")
    return linkage

def update_middle_bogie_linkage():
    # Compute length and angle
    height = params["corner_wheel_asm_height"] + params["steering_asm_height"] + params["rear_steering_mount_neck_height"] + (params["linkage_width"] / 2) - (params["wheel_diameter"] / 2) # h2 + n_y + t/2 + D_w/2
    width = params["rover_length"] / 4 # w
    angle, extended_length = get_linkage_angle_and_extended_length(height, width) #theta, l
    lower_pivot_housing_radius = get_pivot_housing_diameter("lower_") / 2 # r
    length = extended_length - (lower_pivot_housing_radius + (params["middle_wheel_shaft_diameter"] / 2) + (2 * params["linkage_mount_base_length"])) # l - (R + r_ws + 2b)

    # Update linkage file and return map
    linkage = get_linkage_map(length, angle)
    FileHandler.map_to_text_file(linkage, "middle_bogie_linkage.txt")
    return linkage

def update_rear_bogie_linkage():
    # Compute length and angle
    width = params["rover_length"] / 4 # w
    lower_pivot_housing_radius = get_pivot_housing_diameter("lower_") / 2 # r
    length = width - (lower_pivot_housing_radius + (params["wheel_diameter"] / 2) + (params["linkage_width"] / 2) + (2.5 * params["linkage_mount_base_length"])) # w - (r + D_w/2 + n_x/2 + 2b)

    # Update linkage file and return map
    linkage = get_linkage_map(length, 0)
    FileHandler.map_to_text_file(linkage, "rear_bogie_linkage.txt")
    return linkage

## PIVOT HOUSINGS & SPACERS ##

# Takes prefix and angles of housed linkages
def update_pivot_housing(prefix, interior_angle_1, interior_angle_2):
    pivot_housing = {
        "housing_diameter": (get_pivot_housing_diameter(prefix), "mm"),
        "housing_thickness": (params["linkage_thickness"], "mm"),

        "bearing_diameter": (params[prefix + "bearing_diameter"], "mm"),
        "bearing_outer_race_inner_diameter": (params[prefix + "bearing_outer_race_inner_diameter"], "mm"),
        "bearing_thickness": (params[prefix + "bearing_thickness"], "mm"),

        "housing_min_wall_thickness": (params["pivot_housing_min_wall_thickness"], "mm"),
        "housing_bolt_diameter": (params["pivot_housing_bolt_diameter"], "mm"),
        "linkage_separation_angle": (180 - (interior_angle_1 + interior_angle_2), None),
        "bolt_placement_radius": None,
        "num_bolts": (params[prefix + "pivot_housing_num_bolts"], None),

        "linkage_mount_base_width": (params["linkage_width"], "mm"),
        "linkage_mount_base_length": (params["linkage_mount_base_length"], "mm"),
        "linkage_mount_shoulder_depth": (params["linkage_wall_thickness"], "mm"), # TODO add tolerance
        "linkage_mount_bolt_diameter": (params["linkage_mount_bolt_diameter"], "mm"),
        "linkage_mount_bolt_spacing": None,
        "linkage_mount_tongue_length": None
    }
    pivot_housing["bolt_placement_radius"] = ((pivot_housing["bearing_diameter"][0] / 2) + pivot_housing["housing_min_wall_thickness"][0] + (pivot_housing["housing_bolt_diameter"][0] / 2), "mm")
    pivot_housing["linkage_mount_bolt_spacing"] = (BOLT_SPACING_FACTOR * pivot_housing["linkage_mount_bolt_diameter"][0], "mm")
    pivot_housing["linkage_mount_tongue_length"] = (3 * pivot_housing["linkage_mount_bolt_spacing"][0] + 2 * pivot_housing["linkage_mount_bolt_diameter"][0], "mm") # Only accounts for 2 bolts
    
    FileHandler.map_to_text_file(pivot_housing, prefix + "pivot_housing.txt")
    return pivot_housing

# Takes respective pivot_housing map
def update_spacer(prefix, pivot_housing):
    spacer = {
        "outer_diameter": (pivot_housing["housing_diameter"][0], "mm"),
        "inner_diameter": (pivot_housing["bearing_diameter"][0], "mm"),
        "spacer_thickness": None,
        "bolt_diameter": (pivot_housing["housing_bolt_diameter"][0], "mm"),
        "bolt_placement_radius": (pivot_housing["bolt_placement_radius"][0], "mm"),
        "num_bolts": (params[prefix + "pivot_housing_num_bolts"], None)
    }

    if prefix == "upper_":
        thickness = params["middle_wheel_clearance"]
    else:
        thickness = params["upper_shaft_overhang"] + params["middle_wheel_clearance"] + params["middle_wheel_shaft_overhang"]

    spacer["spacer_thickness"] = (thickness, "mm")
    
    FileHandler.map_to_text_file(spacer, prefix + "spacer.txt")
    return spacer

## SHAFTS ##

# Take spacer thicknesses
# Side-effect: prints min bolt length

def update_upper_shaft(upper_spacer_thickness):
    # Compute retention ring positions and length
    ret_ring_1_pos = params["upper_shaft_frame_clearance"] + params["swingarm_thickness"]
    ret_ring_2_pos = ret_ring_1_pos + 2 * params["linkage_thickness"] + upper_spacer_thickness
    ref_length = ret_ring_2_pos + params["upper_shaft_overhang"]
    length = 2 * (ref_length) + params["frame_width"]

    # Print min bolt length
    min_bolt_length = ret_ring_2_pos - params["upper_shaft_frame_clearance"]
    print(f"Min upper bolt length: {min_bolt_length}mm\n")

    # Update file and return map
    shaft = get_shaft_map("upper_")
    shaft["ret_ring_1_pos"] = (ret_ring_1_pos, "mm")
    shaft["ret_ring_2_pos"] = (ret_ring_2_pos, "mm")
    shaft["ref_length"] = (ref_length, "mm")
    shaft["length"] = (length, "mm")

    FileHandler.map_to_text_file(shaft, "upper_shaft.txt")
    return shaft

def update_lower_shaft(upper_spacer_thickness, lower_spacer_thickness):
    # Compute retention ring positions and length
    ret_ring_1_pos = params["lower_shaft_overhang"]
    ret_ring_2_pos = ret_ring_1_pos + params["linkage_thickness"]
    ret_ring_3_pos = ret_ring_2_pos + upper_spacer_thickness 
    ret_ring_4_pos = ret_ring_3_pos + 2 * params["linkage_thickness"] + lower_spacer_thickness 
    length = ret_ring_4_pos + params["lower_shaft_overhang"]
    
    # Print min bolt length
    min_bolt_length = ret_ring_4_pos - ret_ring_3_pos
    print(f"Min lower bolt length: {min_bolt_length}mm\n")

    # Update file and return map
    shaft = get_shaft_map("lower_")
    shaft["ret_ring_1_pos"] = (ret_ring_1_pos, "mm")
    shaft["ret_ring_2_pos"] = (ret_ring_2_pos, "mm")
    shaft["ret_ring_3_pos"] = (ret_ring_3_pos, "mm")
    shaft["ret_ring_4_pos"] = (ret_ring_4_pos, "mm")
    shaft["length"] = (length, "mm")

    FileHandler.map_to_text_file(shaft, "lower_shaft.txt")
    return shaft

## STEERING MOUNTS ##

def update_steering_mount(prefix, offset, angle, pivot_housing):
    steering_mount = {
        "neck_height": (params[prefix + "steering_mount_neck_height"], "mm"),
        "arm_length": (params["linkage_mount_base_length"] + pivot_housing["linkage_mount_tongue_length"][0] + offset, "mm"),
        "angle": (angle, None),
        "width": (params["linkage_width"], "mm"),
        "mount_thickness": (params["linkage_thickness"], "mm")
    }
    steering_mount["fillet_radius"] = (STEERING_MOUNT_FILLET_RADIUS_FACTOR * steering_mount["neck_height"][0], "mm")

    key_list = ["linkage_mount_tongue_length", "linkage_mount_shoulder_depth", "linkage_mount_bolt_diameter", "linkage_mount_bolt_spacing"]
    append_map(key_list, pivot_housing, steering_mount)
    
    FileHandler.map_to_text_file(steering_mount, prefix + "steering_mount.txt")
    return steering_mount 

def update_middle_wheel_mount(pivot_housing):
    middle_wheel_mount = {}
    params_key_list = ["middle_wheel_shaft_diameter", "middle_wheel_shaft_length", "middle_wheel_shaft_overhang", "wheel_diameter", "wheel_thickness", "linkage_thickness", "linkage_width"]
    pivot_key_list = ["linkage_mount_base_length", "linkage_mount_tongue_length", "linkage_mount_shoulder_depth", "linkage_mount_bolt_diameter", "linkage_mount_bolt_spacing"]
    append_map(params_key_list, params_with_units, middle_wheel_mount)
    append_map(pivot_key_list, pivot_housing, middle_wheel_mount)
    FileHandler.map_to_text_file(middle_wheel_mount, "middle_wheel_mount.txt")
    return middle_wheel_mount

## MAIN ##

def main():
    # Validation
    print(f"Rover width is valid: {validate_rover_width()}\n")
    
    # Linkages
    front_rocker_linkage, offset = update_front_rocker_linkage()
    rear_rocker_linkage = update_rear_rocker_linkage()
    middle_bogie_linkage = update_middle_bogie_linkage()
    rear_bogie_linkage = update_rear_bogie_linkage()

    # Pivot housings
    upper_pivot_housing = update_pivot_housing("upper_", front_rocker_linkage["angle"][0], rear_rocker_linkage["angle"][0])
    lower_pivot_housing = update_pivot_housing("lower_", middle_bogie_linkage["angle"][0], rear_bogie_linkage["angle"][0])

    # Spacers
    upper_spacer = update_spacer("upper_", upper_pivot_housing)
    lower_spacer = update_spacer("lower_", lower_pivot_housing)

    # Shafts
    update_upper_shaft(upper_spacer["spacer_thickness"][0])
    update_lower_shaft(upper_spacer["spacer_thickness"][0], lower_spacer["spacer_thickness"][0])

    # Mounts
    update_steering_mount("front_", offset, front_rocker_linkage["angle"][0], upper_pivot_housing)
    update_steering_mount("rear_", 0, 0, lower_pivot_housing)
    update_middle_wheel_mount(lower_pivot_housing)
    
    return

main()