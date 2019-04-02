#!/usr/bin/python
#kate: tab-width 4; indent-width 4; replace-tabs on;
#Creating a new nimbro_fsm2 state using the template state
#Author: Christian Lenz <lenz@ais.uni-bonn.de>

import os
import em
import sys
import rospkg
import copy
import argparse
from catkin_pkg.packages import find_packages

#Find current package
search_path = os.getcwd()
stop_path = '/'
child_path = '.'
package_name = None

# Fid packages under the search path
while search_path != stop_path:
    try:
        pkgs = find_packages(search_path)
    except RuntimeError:
        print "Current folder is not part of a package!"
        sys.exit()

    # Check if the directory is a catkin package
    if child_path in pkgs:
        package_name = pkgs[child_path].name
        break

    # Update search path
    (search_path, child_path) = os.path.split(search_path)

if package_name == None:
    print "Can't find current package"
    sys.exit()

#find nimbro_fsm2 package path to open the template files
rospack = rospkg.RosPack()
try:
    fsm2_path = rospack.get_path('nimbro_fsm2') + "/templates/"
except:
    print "Can't find 'nimbro_fsm2' package. No idea where the templates are :("
    sys.exit()

try:
    package_path = rospack.get_path(package_name)
except:
    print "Can't find '" + package_path + "' package."
    sys.exit()


#argparse defining all optional arguments
argparser = argparse.ArgumentParser(description = "Generate a new nimbro_fsm2 state .cpp and .h file and add the includes into the CMakeLists.txt and the driver file")

argparser.add_argument("name", help="Name for the new state (class name). The lower case version is used as file name (.cpp and .h file)")
argparser.add_argument("-ns", "--name_space", help="Namespace used for the new state. The state files will be placed in a subfoulder")
argparser.add_argument("-fsm", help="Name of the fsm header file. This file will be included in the new state. Default: 'fsm.h'.")
argparser.add_argument("-d", "--driver", help="Name of the driver .cpp file. The new state will be registered  in this file (i.e. add #include). Default: 'driver.cpp'.")
argparser.add_argument("-c", "--only_create", help="Create the .cpp and .h file without registering the state. The fsm.h and driver.cpp files are not needed.", action="store_true")

args = argparser.parse_args()

#dictionary to pass all variables to empy
vars = {
    "state_name" : "TestState",
    "state_ns" : None,
    "fsm" : "fsm",
    "driver" : "driver",
    "package_name" : package_name,
}
if args.name:
    vars["state_name"] = args.name
if args.fsm:
    vars["fsm"] = args.fsm
if args.name_space:
    vars["state_ns"] = args.name_space
if args.driver:
    vars["driver"] = args.driver

print "Creating new nimbro_fsm2 state files:"
print "\t- " + vars["state_name"].lower() + ".cpp"
print "\t- " + vars["state_name"].lower() + ".h"


#get relative fsm.h path.
if vars["state_ns"] != None:
    vars["fsm"] = "../../" + vars["fsm"]
else:
    vars["fsm"] = "../" + vars["fsm"]

fsm_path = package_path + "/src/" + vars["driver"] + ".cpp"
if not os.path.isfile(fsm_path):
    print fsm_path + "  not found"
    sys.exit()

print "Include fsm header '" + vars["fsm"] + ".h'"

#search driver.cpp
driver_path = package_path + "/src/" + vars["driver"] + ".cpp"
if not os.path.isfile(driver_path):
    print driver_path + " not found"
    sys.exit()


#create subfolder if needed and set state file paths
if vars["state_ns"] != None:
    try:
        os.mkdir(package_path + "/src/states/" + vars["state_ns"])
    except OSError:
        print ("Folder '/src/states/%s' already exists." % vars["state_ns"])
    include_path = "states/" + vars["state_ns"] + "/" + vars["state_name"].lower() + ".h"
    state_path = "src/states/" + vars["state_ns"] + "/" + vars["state_name"].lower()

else:
    include_path = "states/" + vars["state_name"].lower() + ".h"
    state_path = "src/states/" + vars["state_name"].lower()


#register state in driver.cpp
with open(driver_path, "r") as in_file:
    buf = in_file.readlines()

    #the include is added after the last '#include' line
    idx = 0
    include = True
    for i in range(len(buf) - 1):
        if "#include" in buf[i]:
            idx = i
        if '#include "' + include_path + '"' in buf[i]:
            include = False
            break
        i+=1

    if include:
        #add the new line and save the file
        print "Register state in driver file '" + vars["driver"] + "'"
        buf[idx] = buf[idx] + '#include "' + include_path + '"\n'
        with open(driver_path, "w") as out_file:
            for line in buf:
                out_file.write(line)
    else:
        print "State was already included in the driver '" + vars["driver"] + "'"




#find CMakeLists.txt and add new state
with open(package_path + "/CMakeLists.txt", "r") as in_file:
    buf = in_file.readlines()

    with open(package_path + "/CMakeLists.txt", "w") as out_file:

        in_ex = False
        for line in buf:
            if "add_executable(" + package_name in line:
                in_ex = True
                print "CMakeLists found add_executable"
            if in_ex and ")" in line:
                print line
                line = "\t" + state_path + ".cpp\n)\n"
                print "CMakeLists found end + added .cpp"
                print line
                in_ex = False

            out_file.write(line)



# create new state files
outcpp = open(state_path + ".cpp", "w")
outheader = open(state_path + ".h", "w")

#copy the variables for both templates
varsh = copy.deepcopy(vars)

#interpret .cpp template
interpretercpp = em.Interpreter(globals = vars, output = outcpp)
try:
    interpretercpp.file(open(fsm2_path + "state.cpp"))

finally:
    interpretercpp.shutdown()

outcpp.close()

#interpret header file
interpreterh = em.Interpreter(globals = varsh, output = outheader)
try:
    interpreterh.file(open(fsm2_path + "state.h"))

finally:
    interpreterh.shutdown()

outheader.close()

