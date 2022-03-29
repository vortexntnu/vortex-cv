def read_yaml_file(filename):
    """
    @parameters filename, filename of the yaml file
    @returns dictionary of each parameter with the corresponding value.
    """

    parameters = {}

    with open(filename) as file:
        lines = file.readlines()
        for line in lines:
            line = line.strip()
            for word in line:
                word = word.replace(" ", "")
                word = word.strip()
                # print(word)

    for line in lines:
        processed = line.strip().split(":")
        key, value = processed[0].strip(), processed[1].strip()
        try:
            if value.find(".") == -1:
                value = int(value)
            else:
                value = float(value)
        except ValueError:
            continue
        parameters[key] = value
    return parameters


# Example:
# print(parameters["hsv_hue_max"])
# print(read_yaml_file("gate_2.yaml")["hsv_hue_max"])
