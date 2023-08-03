import os
from sys import platform
from pathlib import Path
import pathlib
import subprocess
import shutil
import argparse
import time
import urllib.request
from urllib.parse import urlparse

build_with_debug_config = 0

msysPath = "C:/msys64/msys2_shell.cmd"
bazelPath = "C:/ProgramData/chocolatey/bin/bazel.exe"
pythonPath = "%localappdata%\Programs\Python\Python39"

parser = argparse.ArgumentParser(description='pr_mediapipe build script', allow_abbrev=False, formatter_class=argparse.ArgumentDefaultsHelpFormatter, epilog="")
parser.add_argument("--msys2-path", help="Path to msys2_shell.cmd.", default=msysPath)
parser.add_argument("--bazel-path", help="Path to where bazel executable is located.", default=bazelPath)
parser.add_argument("--python39-path", help="Path to where python 3.9 is located.", default=pythonPath)
args,unknown = parser.parse_known_args()
args = vars(args)

if args["msys2_path"]:
	msysPath = args["msys2_path"]
if args["python39_path"]:
	pythonPath = args["python39_path"]
if args["bazel_path"]:
	bazelPath = args["bazel_path"]

########## OpenCV ##########
# Despite what it says in the mediapipe documentation, it currently only works with OpenCV v3.4.10.
# See mediapipe/WORKSPACE "opencv" URL to see which version is actually required.
opencv_dir_base_name = "opencv_mediapipe"
opencv_root = deps_dir +"/" +opencv_dir_base_name
opencv_build = opencv_root +"/build"
opencv_install = opencv_root +"/install"

# Unfortunately that version of OpenCV cannot be built with newer versions than C++11, which means
# it can't be built with Visual Studio 2022 and newer. For this reason we'll use prebuilt binaries on Windows.
os.chdir(deps_dir)
if platform == "linux":
	# TODO: This is untested!
	if not Path(opencv_root).is_dir():
		print_msg(opencv_dir_base_name +" not found, downloading...")
		git_clone("https://github.com/opencv/opencv.git",opencv_dir_base_name)
	os.chdir(opencv_dir_base_name)
	# Version 3.4.1.0
	reset_to_commit("1cc1e6f")

	# OpenCV Contrib
	os.chdir(deps_dir)
	opencv_contrib_root = deps_dir +"/opencv_mediapipe_contrib"
	if not Path(opencv_contrib_root).is_dir():
		print_msg(opencv_dir_base_name +"_contrib not found, downloading...")
		git_clone("https://github.com/opencv/opencv_contrib.git",opencv_dir_base_name +"_contrib")
	os.chdir(opencv_dir_base_name +"_contrib")
	reset_to_commit("5d2cf95") # Has to match OpenCV version

	if build_with_debug_config:
		os.chdir(deps_dir)
		python_dir = deps_dir +"/cpython"
		if not Path(python_dir).is_dir():
			git_clone("https://github.com/python/cpython.git")
		os.chdir("cpython")
		reset_to_commit("878ead1")

		p = subprocess.Popen([python_dir +"/PCbuild/build.bat","-c","Debug"])
		p.communicate()

	# Build and install
	build_config = "Release"
	if build_with_debug_config:
		build_config = "Debug"
	os.chdir(opencv_root)
	mkdir("build",cd=True)
	args = ["-DCXX_STANDARD=11","-DBUILD_opencv_world=ON","-DOPENCV_EXTRA_MODULES_PATH=" +opencv_contrib_root +"/modules","-DBUILD_TESTS=OFF","-DBUILD_PERF_TESTS=OFF","-DBUILD_opencv_apps=OFF","-DBUILD_EXAMPLES=OFF","-DINSTALL_PYTHON_EXAMPLES=OFF","-DINSTALL_C_EXAMPLES=OFF","-DBUILD_opencv_python2=OFF","-DCMAKE_INSTALL_PREFIX=" +opencv_install]
	if build_with_debug_config:
		args += ["-DPYTHON3_DEBUG_LIBRARY:FILEPATH=" +python_dir +"/PCbuild/amd64/python311_d.lib"]
	cmake_configure("..",generator,args)

	cmake_build(build_config)
	cmake_build(build_config,["opencv_world"])
	cmake_build(build_config,["install"])
else:
	if not Path(opencv_root).is_dir():
		print_msg(opencv_dir_base_name +" not found, downloading...")
		os.chdir(deps_dir)
		mkdir(opencv_dir_base_name,cd=True)

		# These binaries originate from https://github.com/opencv/opencv/releases/tag/3.4.10
		http_download("https://github.com/Silverlan/opencv-3.4.10-winx64/releases/download/v3.4.10/opencv-3.4.10-winx64.zip")
		extract("opencv-3.4.10-winx64.zip")

########## mediapipe ##########
os.chdir(deps_dir)
mediapipe_root = deps_dir +"/mediapipe"
if not Path(mediapipe_root).is_dir():
	print_msg("mediapipe not found, downloading...")
	git_clone("https://github.com/google/mediapipe.git")
os.chdir(mediapipe_root)
reset_to_commit("7ba21e9")

# mediapipe_pragma_wrapper
os.chdir(deps_dir)
mediapipe_pragma_wrapper_root = deps_dir +"/mediapipe_pragma_wrapper"
if not Path(mediapipe_pragma_wrapper_root).is_dir():
	print_msg("mediapipe_pragma_wrapper not found, downloading...")
	git_clone("https://github.com/Silverlan/mediapipe_pragma_wrapper.git")
os.chdir(mediapipe_pragma_wrapper_root)
reset_to_commit("d0dba4e")

# We need to make a few changes to the mediapipe files, so we'll apply a patch
os.chdir(mediapipe_root)

p = subprocess.run(["git","apply","--reverse","--check",mediapipe_pragma_wrapper_root +"/mediapipe.patch"])
if p.returncode != 0: # Patch has probably not been applied yet
	print_msg("Applying mediapipe patch...")
	subprocess.run(["git","apply",mediapipe_pragma_wrapper_root +"/mediapipe.patch"],check=True)

# Copy mediapipe_pragma_wrapper to mediapipe
def copy_mp_files(src_dir, dest_dir):
    import shutil
    import os
    for root, dirs, files in os.walk(src_dir):
        # Exclude the ".git" directory
        if ".git" in dirs:
            dirs.remove(".git")

        for file in files:
            src_file = os.path.join(root, file)
            dest_file = os.path.join(dest_dir, os.path.relpath(src_file, src_dir))

            # Create the destination directory if it doesn't exist
            os.makedirs(os.path.dirname(dest_file), exist_ok=True)

            # Copy the file and overwrite if it exists
            shutil.copy2(src_file, dest_file)
print_msg("Copying mediapipe_pragma_wrapper files...")
copy_mp_files(mediapipe_pragma_wrapper_root +"/mediapipe",mediapipe_root +"/mediapipe")

mediapipeTarget = "mediapipe/examples/desktop/mediapipe_pragma_wrapper:mediapipe_pragma_wrapper"

openCvPath = opencv_build
openCvPath = openCvPath.replace("/", "\\").replace("\\", "\\\\")
buildType = "opt"
if build_with_debug_config:
	buildType = "dbg"
bazelRootPath = deps_dir +"/bazel"
bazelRootPath = bazelRootPath.replace("\\", "/")

bazelRcPath = mediapipe_root +"/.bazelrc"
bazelRootEntry = "startup --output_user_root=" +bazelRootPath
strIdx = open(bazelRcPath, 'r').read().find(bazelRootEntry)
if strIdx == -1:
    with open(bazelRcPath, 'a') as file:
        file.write(bazelRootEntry + '\n')

workspaceFile = mediapipe_root +"/WORKSPACE"
openCvEntry = "path = \"C:\\\\opencv\\\\build\","
strIdx = open(workspaceFile, 'r').read().find(openCvEntry)
if strIdx != -1:
    replace_text_in_file(workspaceFile,openCvEntry,"path = \"" +openCvPath +"\",")

bazel_cache_dir = deps_dir +"/_bazel"
bazel_cache_dir = bazel_cache_dir.replace("\\", "/")
script_dir = os.path.dirname(os.path.abspath(__file__))
msys_cmd_python = "SET \"PATH=%PATH%;" +pythonPath +"\""
msys_cmd_bazel = bazelPath +" --output_user_root=\"" +bazel_cache_dir +"\" build -c " +buildType +" --define MEDIAPIPE_DISABLE_GPU=1 " +mediapipeTarget
msys_cmd_bazel += " --experimental_cc_shared_library"

os.chdir(script_dir)
mkdir("temp",cd=True)
# Create temporary bat-script with the commands we need to execute in the msys2 shell
mediapipe_build_file = script_dir +"/temp/build_mediapipe.bat"
mediapipe_build_file = mediapipe_build_file.replace("\\", "/")
print_msg("Generating msys2 batch-script...")
with open(mediapipe_build_file, 'w') as file:
	nmediapipe_root = mediapipe_root
	nmediapipe_root = nmediapipe_root.replace("\\", "/")
	file.write("cd /d \"" +nmediapipe_root +"\"\n")
	file.write(msys_cmd_python +"\n")
	file.write(msys_cmd_bazel +"\n")
	# file.write("sleep 30")

# Build mediapipe wrapper
# Unfortunately mediapipe can only be built with msys2 under Windows.
os.chdir(mediapipe_root)
print_msg("Building mediapipe using msys2...")
subprocess.check_call( [msysPath, "-c", mediapipe_build_file] )

# We have to wait until msys2 has completed building, but unfortunately we can't just wait for the process,
# because msys2 indirectly launches a separate "bash.exe" process, which then executes the actual commands.
# For this reason we have to do a hacky work-around to determine whether it's complete, by simply checking if
# a "bash.exe" process is currently running.
# This means that no other msys2 shell must be running in the background, because the script will wait until all
# of them have been closed.
# We'll also wait 3 seconds initially to ensure that msys2 has had enough time to actually launch "bash.exe".
print("Waiting for mediapipe build to finish...")
time.sleep(3)

# Wait to finish
# Command to check if the process is running
command = 'tasklist /FI "IMAGENAME eq bash.exe"'

# Check if the process is running in a loop
while True:
    # Run the command and capture the output
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)

    # Check the output for the presence of the process
    if 'bash.exe' not in result.stdout.decode():
        break  # Exit the loop if the process has finished

    time.sleep(1)  # Wait for 1 second before checking again
print_msg("Done!")

shutil.rmtree(script_dir +"/temp")

# All the tasks we need have been generated at this point, the only exception being the blendshapes one, which we have to
# download manually.
asset_dir_bin = mediapipe_root +"/bazel-bin/mediapipe/tasks/testdata/vision"
os.chdir(asset_dir_bin)
http_download("https://storage.googleapis.com/mediapipe-assets/face_landmarker_v2_with_blendshapes.task")

cmake_args.append("-DMEDIAPIPE_WRAPPER_BUILD_DIR=" +mediapipe_root +"/bazel-bin/mediapipe/examples/desktop/mediapipe_pragma_wrapper")
cmake_args.append("-DMEDIAPIPE_OPENCV_BUILD_DIR=" +opencv_build +"/x64/vc15/bin")
cmake_args.append("-DMEDIAPIPE_ASSET_DIR=" +asset_dir_bin)

cmake_args.append("-DDEPENDENCY_PRAGMA_MEDIAPIPE_WRAPPER_INCLUDE:PATH=" +mediapipe_root +"/mediapipe/examples/desktop/mediapipe_pragma_wrapper/")
cmake_args.append("-DDEPENDENCY_PRAGMA_MEDIAPIPE_WRAPPER_LIBRARY:FILEPATH=" +mediapipe_root +"/bazel-bin/mediapipe/examples/desktop/mediapipe_pragma_wrapper/mediapipe_pragma_wrapper.if.lib")
