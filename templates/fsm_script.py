#!/usr/bin/python
#kate: tab-width 4; indent-width 4; replace-tabs on;
#Creating a new nimbro_fsm2 package
#Author: Christian Lenz <lenz@ais.uni-bonn.de>

import os
import em
import sys
import rospkg
import copy
import argparse



#Creating directory
def create_folder(path):
    try:
        os.mkdir(path)
    except OSError:
        print ("Folder %s already exists." % path)

#Interpret template file
def interpret_template(template_path, output_path, global_vars):
    output = open(output_path, "w")
    gv = copy.deepcopy(global_vars)
    interpretercmake = em.Interpreter(globals = gv, output = output)
    try:
        interpretercmake.file(open(template_path))
    finally:
        interpretercmake.shutdown()
    output.close()


#find nimbro_fsm2 package path to open the template files
rospack = rospkg.RosPack()
try:
    fsm2_path = rospack.get_path('nimbro_fsm2') + "/templates/"
except:
    print "Can't find nimbro_fsm2 package. No idea where the templates are :("
    sys.exit()

#argparse defining all optional arguments
argparser = argparse.ArgumentParser(description = "Generate a new nimbro_fsm2 state .cpp and .h file at the current path")

argparser.add_argument("name", help="Fsm package name")
argparser.add_argument("-s", "--create_state", help="Create a example state", action="store_true")
argparser.add_argument("-fsm", help="fsm header file name. Default: 'fsm.h'")
argparser.add_argument("-d", "--driver", help="driver class name. Default: 'Driver'.")
argparser.add_argument("--state_name", help="State name. Only used if a new state is created. Default: 'MyState'")
argparser.add_argument("--state_ns", help="State namespace. Only used if a new state is created. Default: no namespace")

args = argparser.parse_args()


package_name = args.name



#Create needed folder
create_folder(package_name)
create_folder(package_name + "/src")
create_folder(package_name + "/src/states")


template_vars = {
    "package_name" : args.name,
    "fsm" : "fsm",
    "driver" : "Driver",
    "create_state" : args.create_state,
    #"state_file" : "test_state",
    "state_name" : "MyState",
    "state_ns" : None,
}

if args.fsm:
    template_vars["fsm"] = args.fsm

if args.driver:
    template_vars["driver"] = args.driver

if args.state_name:
    template_vars["state_name"] = args.state_name

if args.state_ns:
    template_vars["state_ns"] = args.state_ns




#Create CMakeLists
interpret_template(fsm2_path + "CMakeLists_template.txt", package_name + "/CMakeLists.txt", template_vars)

#Create package.xml
interpret_template(fsm2_path + "package_template.txt", package_name + "/package.xml", template_vars)

#Create driver.cpp
interpret_template(fsm2_path + "driver.cpp", package_name + "/src/" + template_vars["driver"].lower() + ".cpp", template_vars)

#Create driver.h
interpret_template(fsm2_path + "driver.h", package_name + "/src/" + template_vars["driver"].lower() + ".h", template_vars)

#Create fsm.h
interpret_template(fsm2_path + "fsm.h", package_name + "/src/" + template_vars["fsm"] + ".h", template_vars)

if args.create_state:

    template_vars["fsm"] = "../" + template_vars["fsm"]
    state_path = package_name + "/src/states/"

    if template_vars["state_ns"]:
        create_folder(package_name + "/src/states/" + template_vars["state_ns"])
        state_path = state_path + template_vars["state_ns"] + "/"

        template_vars["fsm"] = "../" + template_vars["fsm"]


    state_path = state_path + template_vars["state_name"].lower()


    #Create state.cpp
    interpret_template(fsm2_path + "state.cpp", state_path + ".cpp", template_vars)

    #Create state.h
    interpret_template(fsm2_path + "state.h", state_path + ".h", template_vars)










