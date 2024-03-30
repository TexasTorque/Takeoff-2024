from os import listdir
from os.path import isfile, join

paths_dir = "src/main/deploy/pathplanner/paths"
func_name = "pathLoader.preloadPath"

for filename in listdir(paths_dir):
    full_path = join(paths_dir, filename)
    if isfile(full_path):
        pathname = filename.replace(".path", "")
        func_call = func_name + "(\"" + pathname + "\");"
        print(func_call)
