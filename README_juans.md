Basic instructions, programs and scripts for Franka-Emika Panda robot

# Robot

## Enabling Robot

Make sure that the panda computer (i.e. `172.16.222.48`) 'sees' the Control Box.
For that, if not done already (i.e. check ifconfig), run:

```
sudo ifconfig enp176s0  172.16.0.1 netmask 255.255.255.0 up
```

Then, on a browser (Chrome recommended) go to Panda Desk: `https://172.16.0.2/desk/`. Unlock joints and set 'activated' (blue) mode (using black button on desk)

# Repo (General Case)

## Cloning a repo

On the panda computer (i.e. `172.16.222.48`)

```
cd /opt/libfranka
git clone <repo git remote URL>
```

## Compiling and Running C/C++ code:

### General CMakeLists.txt

Make sure that `/opt/libfranka/CMakeLists.txt` is up-to-date.
This means that it should include the following option:

```
option(BUILD_<repo_name_in_capital_letters> "Build <repo_name> code" ON)
if(BUILD_<repo_name_in_capital_letters>)
  add_subdirectory(<repo_name>)
endif()
```
### Repo-specific CMakeLists.txt

Make sure that `/opt/libfranka/<repo_name>/CMakeLists.txt` is up-to-date
This means that the CMake list variable NCSS should contain the entries you need.
For example:

```
set(NCSS
  <c_cpp_program>
)
```
For the program in `/opt/libfranka/<repo_name>/<c_cpp_program>.cpp` to be compiled.

### Compilation

```
cd /opt/libfranka/build
sudo cmake --build .
```

### Running C/C++ code

From `/opt/libfranka/build` run:
```
./<repo_name>/<c_cpp_program> <arg_1> ... <arg_N>
```

## Running Python Scripts

There might be several folders in `/opt/libfranka/<repo_name>/` with python scripts in them. For a script `<script_name>.py` in `<folder_name>` do:

```
cd /opt/libfranka/<repo_name>/<folder_name>
python3 <script_name>.py <arg_1> ... <arg_N>
```


# This Repo (all steps)

## Cloning the repo

On the panda computer (i.e. `172.16.222.48`)

```
cd /opt/libfranka
git clone git@github.com:ncskth/fe_panda.git
```

## Compiling and Running C/C++ code:

### General CMakeLists.txt

Make sure that `/opt/libfranka/CMakeLists.txt` is up-to-date.
This means that it should include the following option:

```
option(BUILD_FE_PANDA> "Build fe_panda code" ON)
if(BUILD_FE_PANDA)
  add_subdirectory(fe_panda)
endif()
```
### Repo-specific CMakeLists.txt

Make sure that `/opt/libfranka/fe_panda/CMakeLists.txt` is up-to-date
This means that the CMake list variable NCSS should contain the entries you need.
For example 'explorer':

```
set(NCSS
  explorer
)
```
For the program in `/opt/libfranka/fe_panda/explorer.cpp` to be compiled.

This program receives over UDP a stream of (x,y,z) coordinates and makes the end-effector follow them.

### Compilation

```
cd /opt/libfranka/build
sudo cmake --build .
```

### Running C/C++ code

From `/opt/libfranka/build` run:
```
./fe_panda/explorer <operating-mode> <gripper-open-close>
```

## Running Python Scripts

In folder `/opt/libfranka/fe_panda/middler` there is a script called `randinator.py`, to run it, do:

```
cd /opt/libfranka/fe_panda/middler
python3 randinator.py
```

This script produces a stream of (x,y,z) coordinates and sends them over UDP for the robot's end-effector to follow.




# Running explorer: Common Issues

## Realtime Scheduling Permission

If explorer starts but prints:
```
libfranka: unable to set realtime scheduling: Operation not permitted
```
grant the executable the capability required by libfranka to enable realtime scheduling:

```
sudo setcap cap_sys_nice+ep /opt/libfranka/build/fe_panda/explorer
```

Verify:
```
getcap /opt/libfranka/build/fe_panda/explorer
```
Expected output:
/opt/libfranka/build/fe_panda/explorer = cap_sys_nice+ep

This step only needs to be performed once after building (or whenever the executable is rebuilt).


## rt_aid.txt Not Found

If the program prints:
```
Failed to open the file rt_aid.txt
```
the executable is being launched from a directory that does not contain rt_aid.txt.
Run explorer from the fe_panda directory instead of the build directory:

```
cd /opt/libfranka/fe_panda
../build/fe_panda/explorer <operating-mode> <gripper-open-close>
```
This ensures that rt_aid.txt is found correctly.