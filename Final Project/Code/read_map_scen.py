
def read_map(filename):
    with open(filename, 'r') as file:
        # Skip the header lines
        for _ in range(4):
            file.readline()

        # Read the map and store it in a 2D array
        map_array = []
        for line in file:
            line = line.strip()
            if line == 'map':
                break
            map_array.append(list(line))

    return map_array


class MapObject:
    def __init__(self, version, filename, height, width, param1, param2, param3, param4, value):
        self.version = int(version)
        self.filename = filename
        self.height = int(height)
        self.width = int(width)
        self.param1 = int(param1)
        self.param2 = int(param2)
        self.param3 = int(param3)
        self.param4 = int(param4)
        self.value = float(value)


def read_map_objects(filename):
    map_objects = []

    with open(filename, 'r') as file:
        file.readline()
        for line in file:
            line = line.strip().split('\t')
            map_object = MapObject(*line)
            map_objects.append(map_object)

    return map_objects
