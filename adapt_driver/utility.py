import os

def y_n_prompt(msg):
    prompt = False
    while 1:
        decision = input(msg)
        if decision == "y" or decision == "Y": 
            prompt = True
            break
        elif decision == "n" or decision == "N":
            break
    return prompt

def get_robot_id(path_to_robot_config):
    dir_path =  os.path.dirname(os.path.realpath(__file__))
    id_path = dir_path + "/" + path_to_robot_config
    id_file_path = id_path + "/robot_id.txt"
    
    # Check if robot_id file exists
    if os.path.exists(id_file_path):
        pass
    else:
        print("ID file does not exist, assuming default ID (0)")
        return "0"
    
    id = open(id_file_path, "r").read()[0]

    # Check if corresponding folder exists
    folders = [name for name in os.listdir(id_path)]

    if id in folders:
        print("Config found with id =", id)
        return id
    else:
        print("ID found (", id, ") but matching config file NOT found -- assuming default ID (0)")
        return "0"